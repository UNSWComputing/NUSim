#include "module/SdkBridge/src/BallCommandReader.hpp"

#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <string>

#include "Odometry.h"
#include "OdometryPubSubTypes.h"
#include "shared/k1/NUSimApi.hpp"
#include "shared/message/Commands.hpp"

namespace k1sim::module::sdkbridge {

    using eprosima::fastdds::dds::DataReader;
    using eprosima::fastdds::dds::SampleInfo;
    using eprosima::fastrtps::types::ReturnCode_t;

    BallCommandReader::BallCommandReader(DdsParticipant& dds, NUClear::Reactor& reactor) : reactor_(reactor) {
        // RELIABLE + KEEP_LAST like the RPC reader: a dropped command would silently skip a shot
        reader_ = dds.create_reader<nav_msgs::msg::dds_::Odometry_PubSubType>(k1sim::nusim::TOPIC_BALL_COMMAND,
                                                                              DdsParticipant::rpc_request_reader_qos(5),
                                                                              this);
    }

    void BallCommandReader::on_data_available(DataReader* reader) {
        if (reader != reader_) {
            return;
        }
        nav_msgs::msg::dds_::Odometry_ msg;
        SampleInfo info;
        while (reader_->take_next_sample(&msg, &info) == ReturnCode_t::RETCODE_OK) {
            if (!info.valid_data) {
                continue;
            }
            const std::string frame = msg.header().frame_id();
            if (frame != "world" && frame != "robot") {
                reactor_.log<NUClear::LogLevel::WARN>("SdkBridge: ball command with unknown frame_id '",
                                                      frame,
                                                      "' (expected world or robot), ignored");
                continue;
            }

            auto cmd   = std::make_unique<k1sim::message::BallCommand>();
            cmd->frame = frame == "robot" ? k1sim::message::BallCommand::Frame::ROBOT
                                          : k1sim::message::BallCommand::Frame::WORLD;
            const auto& p         = msg.pose().pose().position();
            const auto& v         = msg.twist().twist().linear();
            const auto& w         = msg.twist().twist().angular();
            cmd->position         = {p.x(), p.y(), p.z()};
            cmd->velocity         = {v.x(), v.y(), v.z()};
            cmd->angular_velocity = {w.x(), w.y(), w.z()};
            cmd->rolling          = msg.child_frame_id() == "rolling";
            reactor_.emit(std::move(cmd));
        }
    }

}  // namespace k1sim::module::sdkbridge
