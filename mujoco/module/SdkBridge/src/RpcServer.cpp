#include "module/SdkBridge/src/RpcServer.hpp"

#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastrtps/types/TypesBase.h>
#include <nlohmann/json.hpp>

#include "LowCmd.h"
#include "LowCmdPubSubTypes.h"
#include "MotorCmd.h"
#include "RpcReqMsg.h"
#include "RpcReqMsgPubSubTypes.h"
#include "RpcRespMsg.h"
#include "RpcRespMsgPubSubTypes.h"
#include "module/SdkBridge/src/RpcDispatch.hpp"
#include "shared/k1/BoosterApi.hpp"
#include "shared/message/Commands.hpp"

namespace k1sim::module::sdkbridge {

    using eprosima::fastdds::dds::DataReader;
    using eprosima::fastdds::dds::SampleInfo;
    using eprosima::fastrtps::types::ReturnCode_t;

    RpcServer::RpcServer(DdsParticipant& dds, NUClear::Reactor& reactor, int64_t unknown_api_status, int robot_id)
        : dds_(dds), robot_id_(robot_id), reactor_(reactor), unknown_api_status_(unknown_api_status) {

        rpc_resp_writer_ =
            dds.create_writer<booster_msgs::msg::dds_::RpcRespMsg_PubSubType>(k1sim::booster::TOPIC_RPC_RESPONSE,
                                                                              DdsParticipant::state_writer_qos());

        rpc_req_reader_ =
            dds.create_reader<booster_msgs::msg::dds_::RpcReqMsg_PubSubType>(k1sim::booster::TOPIC_RPC_REQUEST,
                                                                             DdsParticipant::rpc_request_reader_qos(),
                                                                             this);

        // rt/joint_ctrl (LowCmd_) — optional passthrough, only honoured by Locomotion in
        // RobotMode::CUSTOM. Reuse the RPC reader's RELIABLE/KEEP_LAST QoS; this topic
        // isn't latency-critical enough to warrant its own tuned profile.
        joint_ctrl_reader_ = dds.create_reader<booster_interface::msg::dds_::LowCmd_PubSubType>(
            k1sim::booster::TOPIC_JOINT_CTRL,
            DdsParticipant::rpc_request_reader_qos(5),
            this);
    }

    RpcServer::~RpcServer() {
        // Delete readers while their listener is still alive, before participant teardown.
        dds_.delete_reader(rpc_req_reader_);
        dds_.delete_reader(joint_ctrl_reader_);
    }

    void RpcServer::on_data_available(DataReader* reader) {
        // A reader can notify during create_reader(), before its member pointer has
        // been assigned. Route using the callback's reader; all other state is already
        // initialised and the reply writer is created before either reader.
        const auto& topic = reader->get_topicdescription()->get_name();
        if (topic == k1sim::booster::TOPIC_RPC_REQUEST) {
            handle_rpc_request(reader);
        }
        else if (topic == k1sim::booster::TOPIC_JOINT_CTRL) {
            handle_joint_ctrl(reader);
        }
    }

    void RpcServer::handle_rpc_request(DataReader* reader) {
        booster_msgs::msg::dds_::RpcReqMsg_ req;
        SampleInfo info;
        while (reader->take_next_sample(&req, &info) == ReturnCode_t::RETCODE_OK) {
            if (!info.valid_data) {
                continue;
            }

            RpcOutcome outcome = dispatch_rpc(req.header(),
                                              req.body(),
                                              current_mode_.load(std::memory_order_relaxed),
                                              unknown_api_status_);

            if (outcome.unknown_api_id) {
                reactor_.log<NUClear::LogLevel::WARN>("SdkBridge: RPC unknown/unparseable api_id",
                                                      outcome.api_id,
                                                      "— replying status",
                                                      outcome.status);
            }

            outcome.action.mode_change.robot_id = robot_id_;
            outcome.action.walk.robot_id        = robot_id_;
            outcome.action.head.robot_id        = robot_id_;
            outcome.action.get_up.robot_id      = robot_id_;
            outcome.action.lie_down.robot_id    = robot_id_;
            outcome.action.visual_kick.robot_id = robot_id_;

            switch (outcome.action.kind) {
                case RpcActionKind::MODE_CHANGE:
                    reactor_.emit(std::make_unique<k1sim::message::ModeChangeRequest>(outcome.action.mode_change));
                    break;
                case RpcActionKind::WALK:
                    reactor_.emit(std::make_unique<k1sim::message::WalkCommand>(outcome.action.walk));
                    break;
                case RpcActionKind::HEAD:
                    reactor_.emit(std::make_unique<k1sim::message::HeadCommand>(outcome.action.head));
                    break;
                case RpcActionKind::GET_UP:
                    reactor_.emit(std::make_unique<k1sim::message::GetUpRequest>(outcome.action.get_up));
                    break;
                case RpcActionKind::LIE_DOWN:
                    reactor_.emit(std::make_unique<k1sim::message::LieDownRequest>(outcome.action.lie_down));
                    break;
                case RpcActionKind::VISUAL_KICK:
                    reactor_.emit(std::make_unique<k1sim::message::VisualKickRequest>(outcome.action.visual_kick));
                    break;
                case RpcActionKind::NONE:
                default: break;
            }

            // Reply IMMEDIATELY, synchronously, in this DDS callback — B1LocoClient
            // blocks up to 1000 ms per call (PROTOCOL.md §3).
            booster_msgs::msg::dds_::RpcRespMsg_ resp;
            resp.uuid(req.uuid());
            resp.header(nlohmann::json{{"status", outcome.status}}.dump());
            resp.body(outcome.response_body);
            rpc_resp_writer_->write(&resp);
        }
    }

    void RpcServer::handle_joint_ctrl(DataReader* reader) {
        booster_interface::msg::dds_::LowCmd_ cmd;
        SampleInfo info;
        while (reader->take_next_sample(&cmd, &info) == ReturnCode_t::RETCODE_OK) {
            if (!info.valid_data) {
                continue;
            }

            auto msg      = std::make_unique<k1sim::message::LowCmdMessage>();
            msg->robot_id = robot_id_;
            msg->cmd_type = static_cast<int>(cmd.cmd_type());
            msg->motors.reserve(cmd.motor_cmd().size());
            for (const auto& m : cmd.motor_cmd()) {
                k1sim::message::MotorCmdData d;
                d.mode   = m.mode();
                d.q      = m.q();
                d.dq     = m.dq();
                d.tau    = m.tau();
                d.kp     = m.kp();
                d.kd     = m.kd();
                d.weight = m.weight();
                msg->motors.push_back(d);
            }
            reactor_.emit(msg);
        }
    }

}  // namespace k1sim::module::sdkbridge
