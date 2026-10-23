#ifndef K1SIM_MODULE_SIMULATION_SIMCORE_HPP
#define K1SIM_MODULE_SIMULATION_SIMCORE_HPP

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mujoco/mujoco.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "shared/k1/JointIndex.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/sim/ModelMap.hpp"
#include "shared/sim/PdController.hpp"
#include "shared/sim/StepController.hpp"

namespace k1sim {

    // SimCore owns the mjModel/mjData and the dedicated physics thread. It is deliberately
    // decoupled from NUClear (no Reactor/Environment dependency) so it can be driven directly
    // by unit tests; module::Simulation is a thin NUClear wrapper around it (config loading,
    // SimHandles/SimStateUpdate wiring, Startup/Shutdown lifecycle).
    //
    // Threading contract: mjData (d()) is only ever touched with mutex() held. The physics
    // thread holds it for control+step+snapshot only; callers (e.g. the viewer) should acquire
    // it briefly. set_controller()/controller() are lock-free (std::atomic<StepController*>)
    // so a controller can be installed after start() without racing the physics thread.
    class SimCore {
    public:
        using StateCallback = std::function<void(std::unique_ptr<message::SimStateUpdate>)>;

        struct Config {
            std::string model_path;                  // resolved via k1sim::config::resolve_path by the caller
            std::string initial_keyframe = "ready";  // keyframe to spawn (and reset) into
            double rtf                   = 1.0;      // real-time factor; 0 = free-run (no pacing sleep)
            int robots                   = 1;        // total K1s; extras are attached copies PD-held at "ready"
            // Per-robot spawn (x, y, yaw), main robot first, from a --game; robots must equal its
            // size. Empty = the main robot spawns at its keyframe and extras on a default grid.
            std::vector<std::array<double, 3>> spawns;
            int state_publish_divisor = 20;    // physics steps per SimStateUpdate
            double resync_threshold   = 0.05;  // seconds behind schedule before the deadline resyncs

            // Ground-contact override, applied to the geom named "floor" after the model loads.
            //
            // Without it the feet do NOT see the floor's declared friction: the foot box geom
            // takes the MuJoCo default (1.0) while the robocup floor declares 0.8, and with equal
            // geom priorities MuJoCo uses the element-wise MAX -- so every NUSim walk to date ran
            // at mu = 1.0, grippier than the scene claims and grippier than anything the robot
            // actually stands on. Enabling this sets the floor's priority to 1 so its numbers
            // govern the contact, which is what the training scene does.
            //
            // This is the knob for the one confirmed hardware variable: the same policy and the
            // same command hold up on carpet and progressively fall on synthetic grass.
            struct Surface {
                bool enabled            = false;
                double friction         = 0.8;   // sliding; mu = tan(slip angle)
                double solref_timeconst = 0.02;  // contact softness; keep >= 2 * model timestep
                double solref_dampratio = 1.0;
            } surface;

            // When non-empty, append one CSV row per published state to this path: per foot the
            // total normal force, the centre of pressure in the foot's own frame, and the sole's
            // pitch/roll. Centre of pressure is the tiptoe measurement -- a sole rolled onto its
            // toe puts every contact at the front edge of the support polygon, which is exactly
            // where the friction budget runs out. Off by default.
            std::string foot_log_path;

            std::array<double, JOINT_COUNT> kp{};                   // PD fallback gains (gains.yaml)
            std::array<double, JOINT_COUNT> kd{};                   // PD fallback gains (gains.yaml)
            std::array<double, JOINT_COUNT> ready_pose_fallback{};  // used only if the model has no
                                                                    // "ready" keyframe
        };

        explicit SimCore(Config config, StateCallback on_state = {});
        ~SimCore();

        SimCore(const SimCore&)            = delete;
        SimCore& operator=(const SimCore&) = delete;

        // Loads the MJCF at config.model_path, builds the ModelMap, allocates mjData, and (if a
        // keyframe named "ready" exists) resets to it and records its joint pose as the PD
        // fallback target; otherwise the fallback target is config.ready_pose_fallback.
        // Throws std::runtime_error on failure (missing file, missing joints/actuators, no free
        // root joint — see ModelMap::build). Must be called exactly once, before start().
        void load_model();

        const mjModel* model() const noexcept {
            return m_;
        }
        mjData* data() const noexcept {
            return d_;
        }
        const ModelMap& model_map() const noexcept {
            return map_;
        }
        std::mutex& mutex() noexcept {
            return mutex_;
        }
        std::atomic<double>& measured_rtf() noexcept {
            return measured_rtf_;
        }

        // Thread-safe; may be called before or after start(), any number of times.
        void set_controller(StepController* controller) noexcept {
            controller_.store(controller, std::memory_order_release);
        }
        StepController* controller() const noexcept {
            return controller_.load(std::memory_order_acquire);
        }

        // Resets mjData back to the state load_model() established (the "ready" keyframe, or
        // zeros if the model has none) — robot pose, ball, velocities, controls. Thread-safe
        // (takes the sim mutex); callable while the physics thread runs. Does NOT reset the
        // attached StepController: it keeps its mode and last commands, mirroring a real robot
        // being picked up and placed back on its start point mid-program.
        void reset();

        // Spawns the physics thread. Requires load_model() to have already succeeded. Idempotent:
        // calling start() again while already running is a no-op.
        void start();

        // Signals the physics thread to stop and joins it. Safe to call multiple times, or if the
        // thread was never started.
        void stop();

        // Frees mjData/mjModel. Call after stop(). Safe to call multiple times.
        void unload();

        uint64_t step_count() const noexcept {
            return step_count_.load(std::memory_order_relaxed);
        }
        uint64_t dropped_deadlines() const noexcept {
            return dropped_deadlines_.load(std::memory_order_relaxed);
        }

    private:
        void physics_loop();
        // Applies Config::surface to the "floor" geom. No-op when the override is disabled or
        // the scene has no geom called "floor". Called by load_model() before mj_makeData.
        void apply_surface_override();
        // Appends one row to Config::foot_log_path. No-op when logging is off. Requires the
        // caller to hold mutex_ (it reads d_->contact).
        void log_foot_state();
        // Puts the extra robot copies at their spawn slots in the ready pose, and the main robot
        // at its game spawn when there is one; must run after every keyframe reset (whose
        // zero-padding would pile the extras at the origin).
        void place_robots();
        // Rests the ball on the field's centre mark for a game. No-op no balls are present in the scene.
        void place_ball_on_centre_mark();
        // Spawn (x, y, z, yaw) of robot k (0 = main): Config::spawns, else the default grid.
        std::array<double, 4> spawn_pose(int k) const;
        // Requires the caller to hold mutex_. Reads d_/map_/controller state into a fresh message.
        std::unique_ptr<message::SimStateUpdate> make_snapshot(uint64_t steps) const;

        Config config_;
        StateCallback on_state_;

        mjModel* m_    = nullptr;
        mjData* d_     = nullptr;
        int reset_key_ = -1;  // keyframe id load_model() reset to; reused by reset()
        ModelMap map_{};
        std::array<double, JOINT_COUNT> ready_target_{};
        PdController pd_{};
        // Joint/actuator index maps for the extra --robots copies (root/sensor fields unused);
        // the physics loop PD-holds each of them at ready_target_ every step.
        std::vector<ModelMap> extra_maps_;

        std::mutex mutex_;
        std::atomic<double> measured_rtf_{0.0};
        std::atomic<StepController*> controller_{nullptr};

        std::atomic<bool> running_{false};
        std::thread thread_;
        std::atomic<uint64_t> step_count_{0};
        std::atomic<uint64_t> dropped_deadlines_{0};

        // Foot-contact CSV (Config::foot_log_path); nullptr when logging is off.
        std::FILE* foot_log_    = nullptr;
        int left_foot_body_id_  = -1;
        int right_foot_body_id_ = -1;

        // Head_2, for SimStateUpdate::head (published as rt/head_pose); -1 if absent.
        int head_body_id_ = -1;

        // The scene ball (body + sphere geom named "ball"), for SimStateUpdate::ball (NUSim ground
        // truth); -1 if the scene has none.
        int ball_body_id_ = -1;
        int ball_geom_id_ = -1;
    };

}  // namespace k1sim

#endif  // K1SIM_MODULE_SIMULATION_SIMCORE_HPP
