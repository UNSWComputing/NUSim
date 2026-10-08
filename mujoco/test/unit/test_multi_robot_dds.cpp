// Exercise the production Simulation -> Locomotion -> SdkBridge path, not a
// synthetic router. Two clients use unchanged Booster topics in separate domains.
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fastdds/dds/core/status/PublicationMatchedStatus.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nuclear>
#include <stdexcept>
#include <string>
#include <thread>

#include "LowCmd.h"
#include "LowCmdPubSubTypes.h"
#include "LowState.h"
#include "LowStatePubSubTypes.h"
#include "Pose.h"
#include "PosePubSubTypes.h"
#include "RpcReqMsg.h"
#include "RpcReqMsgPubSubTypes.h"
#include "RpcRespMsg.h"
#include "RpcRespMsgPubSubTypes.h"
#include "module/Locomotion/src/Locomotion.hpp"
#include "module/SdkBridge/src/SdkBridge.hpp"
#include "module/Simulation/src/Simulation.hpp"
#include "shared/k1/BoosterApi.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/util/Config.hpp"

namespace {

    using namespace eprosima::fastdds::dds;
    using booster_interface::msg::dds_::LowCmd_;
    using booster_interface::msg::dds_::LowState_;
    using booster_msgs::msg::dds_::RpcReqMsg_;
    using booster_msgs::msg::dds_::RpcRespMsg_;
    using geometry_msgs::msg::dds_::Pose_;
    using k1sim::module::sdkbridge::DdsParticipant;

    void require(bool condition, const char* label) {
        if (!condition) {
            throw std::runtime_error(label);
        }
    }

    template <typename F>
    void wait_for(F ready, const char* label, std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        do {
            if (ready()) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } while (std::chrono::steady_clock::now() < deadline);
        throw std::runtime_error(label);
    }

    struct TestConfig {
        std::filesystem::path path;
        TestConfig() {
            char pattern[]      = "/tmp/nusim-independent-XXXXXX";
            const char* created = mkdtemp(pattern);
            require(created != nullptr, "temporary config directory failed");
            path                = created;
            const auto original = k1sim::config::config_dir();
            for (const auto* name : {"simulation.yaml", "locomotion.yaml", "gains.yaml"}) {
                std::filesystem::copy_file(original / name, path / name);
            }
            // Isolate test traffic from production domains 0..5 and the DDS transport test.
            std::ofstream(path / "dds.yaml") << "domain: 60\nudp_only: true\n";
            k1sim::cli().config_dir = path.string();
            k1sim::cli().robots     = 2;
            k1sim::cli().headless   = true;
        }
        ~TestConfig() {
            std::filesystem::remove_all(path);
        }
    };

    struct Client {
        DdsParticipant dds;
        DataWriter* request;
        DataWriter* command;
        DataReader* response;
        DataReader* sensors;
        DataReader* head;
        LowState_ latest;
        Pose_ latest_head;
        bool seen      = false;
        bool head_seen = false;
        explicit Client(int domain) : dds(domain, true) {
            request =
                dds.create_writer<booster_msgs::msg::dds_::RpcReqMsg_PubSubType>(k1sim::booster::TOPIC_RPC_REQUEST,
                                                                                 DdsParticipant::state_writer_qos());
            command =
                dds.create_writer<booster_interface::msg::dds_::LowCmd_PubSubType>(k1sim::booster::TOPIC_JOINT_CTRL,
                                                                                   DdsParticipant::state_writer_qos());
            response = dds.create_reader<booster_msgs::msg::dds_::RpcRespMsg_PubSubType>(
                k1sim::booster::TOPIC_RPC_RESPONSE,
                DdsParticipant::rpc_request_reader_qos(),
                nullptr);
            sensors = dds.create_reader<booster_interface::msg::dds_::LowState_PubSubType>(
                k1sim::booster::TOPIC_LOW_STATE,
                DdsParticipant::rpc_request_reader_qos(),
                nullptr);
            head =
                dds.create_reader<geometry_msgs::msg::dds_::Pose_PubSubType>(k1sim::booster::TOPIC_HEAD_POSE,
                                                                             DdsParticipant::rpc_request_reader_qos(),
                                                                             nullptr);
            require(request && command && response && sensors && head, "DDS client endpoints failed");
        }
        bool discovered() {
            PublicationMatchedStatus rpc_status, cmd_status;
            request->get_publication_matched_status(rpc_status);
            command->get_publication_matched_status(cmd_status);
            return rpc_status.current_count > 0 && cmd_status.current_count > 0;
        }
        void read_sensors() {
            SampleInfo info;
            LowState_ state;
            while (sensors->take_next_sample(&state, &info) == ReturnCode_t::RETCODE_OK) {
                if (info.valid_data) {
                    latest = state;
                    seen   = true;
                }
            }
            Pose_ pose;
            while (head->take_next_sample(&pose, &info) == ReturnCode_t::RETCODE_OK) {
                if (info.valid_data) {
                    latest_head = pose;
                    head_seen   = true;
                }
            }
        }
        RpcRespMsg_ rpc(int api, const std::string& body, const std::string& uuid) {
            RpcReqMsg_ req;
            req.uuid(uuid);
            req.header("{\"api_id\":" + std::to_string(api) + "}");
            req.body(body);
            request->write(&req);
            RpcRespMsg_ result;
            wait_for(
                [&] {
                    SampleInfo info;
                    RpcRespMsg_ reply;
                    while (response->take_next_sample(&reply, &info) == ReturnCode_t::RETCODE_OK) {
                        if (info.valid_data && reply.uuid() == uuid) {
                            result = reply;
                            return true;
                        }
                    }
                    return false;
                },
                "RPC reply timeout",
                std::chrono::milliseconds(1000));
            require(result.header() == "{\"status\":0}", "RPC status changed");
            return result;
        }
        void joints(double yaw) {
            LowCmd_ cmd;
            cmd.cmd_type(booster_interface::msg::dds_::SERIAL);
            const auto gains = k1sim::config::load("gains.yaml");
            cmd.motor_cmd().resize(k1sim::JOINT_COUNT);
            for (std::size_t j = 0; j < k1sim::JOINT_COUNT; ++j) {
                cmd.motor_cmd()[j].mode(1);
                cmd.motor_cmd()[j].q(j == k1sim::HeadYaw ? static_cast<float>(yaw)
                                                         : gains["ready_pose"][j].as<float>());
                cmd.motor_cmd()[j].kp(gains["kp"][j].as<float>());
                cmd.motor_cmd()[j].kd(gains["kd"][j].as<float>());
            }
            command->write(&cmd);
        }
    };

    struct RunningPlant {
        NUClear::PowerPlant& plant;
        std::thread worker;
        explicit RunningPlant(NUClear::PowerPlant& value) : plant(value), worker([&value] { value.start(); }) {}
        ~RunningPlant() {
            plant.shutdown();
            worker.join();
        }
    };

}  // namespace

int main() {
    try {
        TestConfig test_config;
        NUClear::Configuration config;
        config.default_pool_concurrency = 4;
        NUClear::PowerPlant plant(config);
        plant.install<NUClear::extension::ChronoController>();
        plant.install<k1sim::module::Simulation>();
        plant.install<k1sim::module::Locomotion>();
        plant.install<k1sim::module::SdkBridge>();
        Client first(60), second(61);
        RunningPlant running(plant);
        wait_for(
            [&] {
                first.read_sensors();
                second.read_sensors();
                return first.discovered() && second.discovered() && first.seen && second.seen;
            },
            "two robots did not publish sensors/discover command endpoints");
        first.rpc(k1sim::booster::ROTATE_HEAD, "{\"pitch\":0.1,\"yaw\":0.4}", "head-first");
        second.rpc(k1sim::booster::ROTATE_HEAD, "{\"pitch\":0.2,\"yaw\":-0.4}", "head-second");
        wait_for(
            [&] {
                first.read_sensors();
                second.read_sensors();
                return first.latest.motor_state_serial().size() == k1sim::JOINT_COUNT
                       && second.latest.motor_state_serial().size() == k1sim::JOINT_COUNT
                       && first.latest.motor_state_serial()[k1sim::HeadYaw].q() > 0.25
                       && second.latest.motor_state_serial()[k1sim::HeadYaw].q() < -0.25 && first.head_seen
                       && second.head_seen && first.latest_head.orientation().z() > 0.1
                       && second.latest_head.orientation().z() < -0.1;
            },
            "head commands/poses crossed robot domains");
        first.rpc(k1sim::booster::CHANGE_MODE, "{\"mode\":3}", "mode-first");
        wait_for(
            [&] {
                return first.rpc(k1sim::booster::GET_MODE, "", "same-uuid").body() == "{\"mode\":3}"
                       && second.rpc(k1sim::booster::GET_MODE, "", "same-uuid").body() == "{\"mode\":1}";
            },
            "GetMode reply/cache crossed robot domains");
        second.rpc(k1sim::booster::CHANGE_MODE, "{\"mode\":3}", "mode-second");
        wait_for([&] { return second.rpc(k1sim::booster::GET_MODE, "", "custom-second").body() == "{\"mode\":3}"; },
                 "second robot could not enter CUSTOM");
        // Switch signs from the head RPCs to prove LowCmd reaches each separate controller.
        first.joints(-0.4);
        second.joints(0.4);
        wait_for(
            [&] {
                first.read_sensors();
                second.read_sensors();
                return first.latest.motor_state_serial()[k1sim::HeadYaw].q() < -0.25
                       && second.latest.motor_state_serial()[k1sim::HeadYaw].q() > 0.25;
            },
            "joint commands or sensor streams crossed robot domains");
        std::puts("production two-robot DDS head/joint/mode/RPC isolation passed");
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
    return 0;
}
