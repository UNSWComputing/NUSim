#ifndef K1SIM_MODULE_SDKBRIDGE_HPP
#define K1SIM_MODULE_SDKBRIDGE_HPP

#include <cstdint>
#include <memory>
#include <mutex>
#include <nuclear>
#include <vector>

#include "module/SdkBridge/src/BallCommandReader.hpp"
#include "module/SdkBridge/src/DdsParticipant.hpp"
#include "module/SdkBridge/src/GroundTruthPublisher.hpp"
#include "module/SdkBridge/src/RpcServer.hpp"
#include "module/SdkBridge/src/StatePublisher.hpp"

namespace k1sim::module {

    // The Booster SDK compatibility surface: FastDDS publishers for
    // rt/low_state, rt/odometer_state, rt/fall_down, rt/battery_state,
    // rt/button_event and the LocoApi RPC server (rt/LocoApiTopicReq/Resp).
    // See module/SdkBridge/PROTOCOL.md for the full wire contract this implements.
    class SdkBridge : public NUClear::Reactor {
    public:
        explicit SdkBridge(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct RobotConnection {
            int robot_id;
            bool seen_state           = false;
            uint64_t last_reset_count = 0;
            uint64_t last_step_count  = 0;
            std::unique_ptr<sdkbridge::DdsParticipant> dds;
            std::unique_ptr<sdkbridge::StatePublisher> publisher;
            std::unique_ptr<sdkbridge::RpcServer> rpc;
            // NUSim-only test surface (PROTOCOL.md §6). Declared last so they are destroyed before the
            // participant. Ground truth is this robot's own state: another robot's pose never reaches
            // this domain on rt/nusim/gt/robot.
            std::unique_ptr<sdkbridge::GroundTruthPublisher> ground_truth;
            std::unique_ptr<sdkbridge::BallCommandReader> ball_command_reader;
        };
        // Serialises Startup, state/battery publication and Shutdown. DDS listener
        // callbacks do not take this mutex: they enqueue commands and reply directly.
        std::mutex connections_mutex_;
        std::vector<RobotConnection> connections_;
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_SDKBRIDGE_HPP
