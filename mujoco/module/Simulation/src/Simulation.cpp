#include "module/Simulation/src/Simulation.hpp"

#include <cmath>
#include <cstddef>
#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "shared/CliOptions.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/message/Commands.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/Config.hpp"
#include "shared/util/RobotRoster.hpp"

namespace k1sim::module {

    namespace {

        SimCore::Config build_sim_config() {
            auto sim_cfg   = config::load("simulation.yaml");
            auto gains_cfg = config::load("gains.yaml");

            SimCore::Config cfg;
            if (cli().game > 0) {
                config::Game game = config::game(sim_cfg, cli().game, cli().on_field_positions);
                cfg.model_path    = config::field_scene(sim_cfg, game.field);
                cfg.robots        = static_cast<int>(game.spawns.size());
                cfg.spawns        = std::move(game.spawns);
            }
            else {
                cfg.model_path = config::field_scene(sim_cfg, cli().field);
                cfg.robots     = cli().robots;
            }
            cfg.identities = config::robot_roster();
            cfg.initial_keyframe =
                !cli().keyframe.empty() ? cli().keyframe : sim_cfg["initial_keyframe"].as<std::string>("ready");
            // CliOptions.rtf < 0 means "use config"; the config's real_time_factor may itself be 0
            // (free-run) — SimCore treats rtf <= 0 as free-run.
            cfg.rtf                   = cli().rtf >= 0.0 ? cli().rtf : sim_cfg["real_time_factor"].as<double>(1.0);
            cfg.state_publish_divisor = sim_cfg["state_publish_divisor"].as<int>(20);
            cfg.resync_threshold      = sim_cfg["resync_threshold"].as<double>(0.05);

            const auto surface           = sim_cfg["surface"];
            cfg.surface.enabled          = surface["enabled"].as<bool>(false);
            cfg.surface.friction         = surface["friction"].as<double>(0.8);
            cfg.surface.solref_timeconst = surface["solref_timeconst"].as<double>(0.02);
            cfg.surface.solref_dampratio = surface["solref_dampratio"].as<double>(1.0);

            cfg.foot_log_path = sim_cfg["foot_log"].as<std::string>("");

            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                cfg.kp[i]                  = gains_cfg["kp"][i].as<double>();
                cfg.kd[i]                  = gains_cfg["kd"][i].as<double>();
                cfg.ready_pose_fallback[i] = gains_cfg["ready_pose"][i].as<double>();
            }
            return cfg;
        }

    }  // namespace

    Simulation::Simulation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Constructed here (not inside on<Startup>) — see the header comment on sim_.
        SimCore::Config sim_config = build_sim_config();
        const std::string scene    = sim_config.model_path;
        sim_                       = std::make_unique<SimCore>(
            std::move(sim_config),
            [this](std::unique_ptr<message::SimStateUpdate> state) { emit(state); },
            [this](std::unique_ptr<message::RobotStatesUpdate> states) { emit(states); });

        on<Startup>().then([this, scene] {
            sim_->load_model();

            auto handles          = std::make_unique<message::SimHandles>();
            handles->model        = sim_->model();
            handles->data         = sim_->data();
            handles->mutex        = &sim_->mutex();
            handles->measured_rtf = &sim_->measured_rtf();
            handles->reset_world  = [this] { sim_->reset(); };
            handles->placed_robot = [this](int id) { sim_->placed_robot_locked(id); };
            emit(handles);

            log<NUClear::LogLevel::INFO>("Simulation ready (scene",
                                         scene,
                                         "— MuJoCo",
                                         mj_versionString(),
                                         "— nq:",
                                         sim_->model()->nq,
                                         "nu:",
                                         sim_->model()->nu,
                                         ") — starting physics thread (PD-to-ready fallback until a "
                                         "controller attaches)");

            // Start immediately with the PD fallback engaged; if Locomotion's ControllerHandle
            // arrives it is attached under the physics lock — no
            // need to wait for it (Locomotion may not even be installed, e.g. in unit tests).
            sim_->start();
        });

        on<Trigger<message::ControllerHandle>>().then([this](const message::ControllerHandle& handle) {
            if (handle.owner) {
                sim_->set_robot_controller(handle.robot_id, handle.owner);
            }
            else if (handle.robot_id == 1) {
                sim_->set_controller(handle.controller);
            }
            log<NUClear::LogLevel::INFO>("Simulation: controller attached for robot", handle.robot_id);
        });

        // Opt-in loopback-only scenario control; absent in normal simulator runs.
        const int scenario_port = config::load("simulation.yaml")["scenario_port"].as<int>(0);
        if (scenario_port) {
            on<UDP, Single>(scenario_port).then([this](const UDP::Packet& packet) {
                if (packet.remote.address != "127.0.0.1")
                    return;
                try {
                    const auto request = nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
                    const auto action  = request.at("action").get<std::string>();
                    if (action == "reset")
                        sim_->reset();
                    else if (action == "fall")
                        sim_->topple_robot(request.at("robot_id").get<int>());
                    else if (action == "record") {
                        auto video      = std::make_unique<message::VideoRecordRequest>();
                        video->path     = request.at("path").get<std::string>();
                        video->duration = request.at("duration").get<double>();
                        if (video->path.empty() || !std::isfinite(video->duration) || video->duration <= 0.)
                            return;
                        emit(std::move(video));
                    }
                    else if (action == "ball") {
                        const double x = request.at("x"), y = request.at("y");
                        if (!std::isfinite(x) || !std::isfinite(y))
                            return;
                        sim_->relocate_ball(x, y);
                    }
                    else
                        return;
                    log<NUClear::LogLevel::INFO>("SCENARIO applied", request.dump());
                }
                catch (const std::exception& e) {
                    log<NUClear::LogLevel::WARN>("SCENARIO rejected", e.what());
                }
            });
        }
        on<Every<1, std::chrono::seconds>>().then([this] {
            log<NUClear::LogLevel::INFO>("SIM_METRICS rtf",
                                         sim_->measured_rtf().load(),
                                         "physics_step_ms",
                                         sim_->physics_step_ms(),
                                         "dropped_deadlines",
                                         sim_->dropped_deadlines());
        });

        // Viewer Backspace restores the world and clears every controller.
        on<Trigger<message::SimResetRequest>>().then([this] {
            sim_->reset();
            log<NUClear::LogLevel::INFO>("Simulation: state reset to startup keyframe");
        });

        on<Trigger<message::RobotResetRequest>>().then([this](const message::RobotResetRequest& req) {
            try {
                sim_->reset_robot(req.robot_id);
                log<NUClear::LogLevel::INFO>("Simulation: reset robot", req.robot_id);
            }
            catch (const std::exception& error) {
                log<NUClear::LogLevel::WARN>("Simulation: robot reset rejected:", error.what());
            }
        });

        // Base-pose heartbeat: one INFO line every ~5 s of sim time (updates arrive at
        // 50 Hz), so headless runs show whether the robot is actually moving.
        on<Trigger<message::SimStateUpdate>>().then([this](const message::SimStateUpdate& s) {
            if (s.sim_time >= next_pose_log_) {
                next_pose_log_ = s.sim_time + 5.0;
                log<NUClear::LogLevel::INFO>("Simulation: t =",
                                             s.sim_time,
                                             "s, base x =",
                                             s.base.x,
                                             "y =",
                                             s.base.y,
                                             "z =",
                                             s.base.z,
                                             ", mode =",
                                             s.mode);
            }
        });

        on<Shutdown>().then([this] {
            log<NUClear::LogLevel::INFO>("Simulation shutting down (measured RTF",
                                         sim_->measured_rtf().load(),
                                         ", dropped deadlines:",
                                         sim_->dropped_deadlines(),
                                         ")");
            sim_->stop();
            sim_->unload();
        });
    }

}  // namespace k1sim::module
