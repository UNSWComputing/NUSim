#include "module/Locomotion/src/Locomotion.hpp"

#include "shared/k1/BoosterApi.hpp"
#include "shared/message/Commands.hpp"
#include "shared/util/Config.hpp"
#include "shared/util/RobotRoster.hpp"

namespace k1sim::module {

    using message::ControllerHandle;
    using message::GetUpRequest;
    using message::HeadCommand;
    using message::LieDownRequest;
    using message::LowCmdMessage;
    using message::ModeChangeRequest;
    using message::VisualKickRequest;
    using message::WalkCommand;

    Locomotion::Locomotion(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Construct before Startup: DDS callbacks and controller attachment may run
        // concurrently with Startup reactions in other modules.
        const auto locomotion_cfg = config::load("locomotion.yaml");
        const auto gains_cfg      = config::load("gains.yaml");
        for (const auto& robot : config::robot_roster()) {
            controllers_.push_back(
                std::make_shared<LocomotionController>(locomotion_cfg, gains_cfg, robot_prefix(robot.model_index)));
        }
        on<Startup>().then([this] {
            for (std::size_t i = 0; i < controllers_.size(); ++i) {
                auto handle        = std::make_unique<ControllerHandle>();
                handle->controller = controllers_[i].get();
                handle->robot_id   = static_cast<int>(i) + 1;
                handle->owner      = controllers_[i];
                emit(handle);
            }
            log<NUClear::LogLevel::INFO>("Locomotion ready — independent controllers:", controllers_.size());
        });

        on<Trigger<HeadCommand>>().then([this](const HeadCommand& cmd) {
            if (auto* controller = controller_for(cmd.robot_id)) {
                controller->set_head_command(cmd.pitch, cmd.yaw);
            }
        });

        on<Trigger<ModeChangeRequest>>().then([this](const ModeChangeRequest& req) {
            if (auto* controller = controller_for(req.robot_id)) {
                log<NUClear::LogLevel::INFO>("ChangeMode requested: robot", req.robot_id, "mode", req.mode);
                controller->request_mode_change(req.mode);
            }
        });

        on<Trigger<LowCmdMessage>>().then([this](const LowCmdMessage& cmd) {
            if (auto* controller = controller_for(cmd.robot_id)) {
                controller->set_low_cmd(cmd.cmd_type, cmd.motors);
            }
        });

        // Locomotion policies (walk, get-up, lie-down, kick) moved to the NUbots_K1 side;
        // they arrive as LowCmd servo targets in CUSTOM mode. The old high-level RPCs stay
        // on the wire for SDK compatibility but are ignored with a warning.
        on<Trigger<WalkCommand>>().then([this](const WalkCommand&) { warn_once(walk_warned_, "Move"); });
        on<Trigger<GetUpRequest>>().then([this](const GetUpRequest&) { warn_once(getup_warned_, "GetUp"); });
        on<Trigger<LieDownRequest>>().then([this](const LieDownRequest&) { warn_once(liedown_warned_, "LieDown"); });
        on<Trigger<VisualKickRequest>>().then(
            [this](const VisualKickRequest&) { warn_once(kick_warned_, "VisualKick"); });

        on<Every<1, std::chrono::seconds>>().then([this] {
            for (std::size_t i = 0; i < controllers_.size(); ++i)
                log<NUClear::LogLevel::INFO>("CONTROL_METRICS robot",
                                             i + 1,
                                             "mailbox_to_apply_ms",
                                             controllers_[i]->command_latency_ms(),
                                             "samples",
                                             controllers_[i]->command_samples());
        });
        on<Shutdown>().then([this] { log<NUClear::LogLevel::INFO>("Locomotion shutting down"); });
    }

    LocomotionController* Locomotion::controller_for(int robot_id) {
        if (robot_id < 1 || robot_id > static_cast<int>(controllers_.size())) {
            log<NUClear::LogLevel::WARN>("Locomotion: ignoring unknown robot", robot_id);
            return nullptr;
        }
        return controllers_[robot_id - 1].get();
    }

    void Locomotion::warn_once(std::atomic<bool>& flag, const char* rpc) {
        if (!flag.exchange(true, std::memory_order_relaxed)) {
            log<NUClear::LogLevel::WARN>(rpc,
                                         "RPC received, but locomotion policies live in NUbots_K1 now; "
                                         "ignored (drive the robot with CUSTOM mode + rt/joint_ctrl)");
        }
    }

}  // namespace k1sim::module
