#include "module/Locomotion/src/Locomotion.hpp"

#include "shared/k1/BoosterApi.hpp"
#include "shared/message/Commands.hpp"
#include "shared/util/Config.hpp"

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

        on<Startup>().then([this] {
            auto locomotion_cfg = config::load("locomotion.yaml");
            auto gains_cfg      = config::load("gains.yaml");

            try {
                controller_ = std::make_unique<LocomotionController>(locomotion_cfg, gains_cfg);
            }
            catch (const std::exception& e) {
                log<NUClear::LogLevel::FATAL>("Locomotion: failed to build the mode controller:", e.what());
                throw;
            }

            emit(std::make_unique<ControllerHandle>(ControllerHandle{controller_.get()}));
            if (controller_->can_walk()) {
                log<NUClear::LogLevel::INFO>("Locomotion ready - built-in walk policy loaded: WALKING / SOCCER mode obey Move");
            }
            else {
                if (!controller_->walk_policy_error().empty()) {
                    log<NUClear::LogLevel::ERROR>("Walk policy not loaded:", controller_->walk_policy_error());
                }
                log<NUClear::LogLevel::INFO>("Locomotion ready (servo-command listener; policies live in NUbots_K1)");
            }
            emit(std::make_unique<message::LocomotionInfo>(message::LocomotionInfo{controller_->can_walk()}));
        });

        on<Trigger<HeadCommand>>().then(
            [this](const HeadCommand& cmd) { controller_->set_head_command(cmd.pitch, cmd.yaw); });

        on<Trigger<ModeChangeRequest>>().then([this](const ModeChangeRequest& req) {
            log<NUClear::LogLevel::INFO>("ChangeMode requested: mode", req.mode);
            controller_->request_mode_change(req.mode);
        });

        on<Trigger<LowCmdMessage>>().then(
            [this](const LowCmdMessage& cmd) { controller_->set_low_cmd(cmd.cmd_type, cmd.motors); });

        // Locomotion policies (walk, get-up, lie-down, kick) moved to the NUbots_K1 side;
        // they arrive as LowCmd servo targets in CUSTOM mode. The old high-level RPCs stay
        // on the wire for SDK compatibility but are ignored with a warning.
        on<Trigger<WalkCommand>>().then([this](const WalkCommand& cmd) {
            if (controller_->can_walk()) {
                controller_->set_walk_command(cmd.vx, cmd.vy, cmd.vyaw);
            }
            else {
                warn_once(walk_warned_, "Move");
            }
        });
        on<Trigger<GetUpRequest>>().then([this](const GetUpRequest&) { warn_once(getup_warned_, "GetUp"); });
        on<Trigger<LieDownRequest>>().then([this](const LieDownRequest&) { warn_once(liedown_warned_, "LieDown"); });
        on<Trigger<VisualKickRequest>>().then(
            [this](const VisualKickRequest&) { warn_once(kick_warned_, "VisualKick"); });

        on<Shutdown>().then([this] { log<NUClear::LogLevel::INFO>("Locomotion shutting down"); });
    }

    void Locomotion::warn_once(bool& flag, const char* rpc) {
        if (!flag) {
            flag = true;
            log<NUClear::LogLevel::WARN>(rpc,
                                         "RPC received, but locomotion policies live in NUbots_K1 now; "
                                         "ignored (drive the robot with CUSTOM mode + rt/joint_ctrl)");
        }
    }

}  // namespace k1sim::module
