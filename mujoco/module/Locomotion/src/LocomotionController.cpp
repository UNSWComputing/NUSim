#include "module/Locomotion/src/LocomotionController.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "module/Locomotion/src/LocoMath.hpp"

namespace k1sim::module {

    namespace {

        // SDK-documented RotateHead limits. Only honoured in PREPARE (CUSTOM commands
        // the head through LowCmd like every other joint).
        constexpr double kHeadPitchMin = -0.3;
        constexpr double kHeadPitchMax = 1.0;
        constexpr double kHeadYawLimit = 0.785;

        void apply_head_target(std::array<double, JOINT_COUNT>& q_ref, double pitch, double yaw) {
            q_ref[JointIndexK1::HeadPitch] = std::clamp(pitch, kHeadPitchMin, kHeadPitchMax);
            q_ref[JointIndexK1::HeadYaw]   = std::clamp(yaw, -kHeadYawLimit, kHeadYawLimit);
        }

        std::array<double, JOINT_COUNT> load_array(const YAML::Node& node) {
            std::array<double, JOINT_COUNT> arr{};
            if (!node) {
                return arr;
            }
            const auto values = node.as<std::vector<double>>();
            for (std::size_t i = 0; i < JOINT_COUNT && i < values.size(); ++i) {
                arr[i] = values[i];
            }
            return arr;
        }

    }  // namespace

    LocomotionController::LocomotionController(const YAML::Node& locomotion_cfg,
                                               const YAML::Node& gains_cfg,
                                               std::string prefix)
        : prefix_(std::move(prefix)) {
        low_cmd_timeout_    = locomotion_cfg["low_cmd_timeout"].as<double>(0.);
        prepare_blend_time_ = locomotion_cfg["prepare_blend_time"].as<double>(1.0);

        const auto fall_node = locomotion_cfg["fall"];
        falling_tilt_        = fall_node["falling_tilt"].as<double>(0.35);
        fallen_tilt_         = fall_node["fallen_tilt"].as<double>(1.0);
        falling_gyro_        = fall_node["falling_gyro"].as<double>(1.0);
        fallen_height_       = fall_node["fallen_height"].as<double>(0.35);

        ready_pose_ = load_array(gains_cfg["ready_pose"]);
        pd_.kp      = load_array(gains_cfg["kp"]);
        pd_.kd      = load_array(gains_cfg["kd"]);

        // The real robot boots into DAMPING (limp until an operator sends Prepare), but a
        // sim robot spawns standing at the ready pose -- idling in DAMPING just collapses
        // it before any client connects. Default to holding PREPARE until the first
        // ChangeMode arrives.
        const std::string initial_mode = locomotion_cfg["initial_mode"].as<std::string>("prepare");
        if (initial_mode == "prepare") {
            initial_mode_ = booster::PREPARE;
            request_mode_change(initial_mode_);
        }
        else if (initial_mode != "damping") {
            throw std::runtime_error("config/locomotion.yaml: unknown initial_mode '" + initial_mode
                                     + "' (expected 'prepare' or 'damping')");
        }
    }

    void LocomotionController::reset() {
        // The caller holds the physics lock, so only mailbox writers can run concurrently.
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        head_pitch_ = head_yaw_ = 0.0;
        low_cmd_type_           = 1;
        low_cmd_motors_.clear();
        requested_mode_ = initial_mode_;
        ++mode_seq_;
        stale_hold_             = false;
        state_                  = State::Damping;
        custom_parallel_warned_ = false;
        custom_low_cmd_seen_    = false;
        custom_entry_step_      = false;
        prepare_start_time_     = 0.0;
        mode_.store(initial_mode_, std::memory_order_relaxed);
        fall_state_.store(booster::IS_READY, std::memory_order_relaxed);
    }

    void LocomotionController::ensure_initialized(const mjModel* m) {
        if (initialized_) {
            return;
        }
        map_         = std::make_unique<ModelMap>(ModelMap::build(m, prefix_));
        initialized_ = true;
    }

    std::array<double, JOINT_COUNT> LocomotionController::current_q(mjData* d) const {
        std::array<double, JOINT_COUNT> q{};
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            q[i] = d->qpos[map_->qpos_adr[i]];
        }
        return q;
    }

    LocomotionController::CommandSnapshot LocomotionController::snapshot_command() const {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        CommandSnapshot snap;
        snap.head_pitch     = head_pitch_;
        snap.head_yaw       = head_yaw_;
        snap.mode_seq       = mode_seq_;
        snap.requested_mode = requested_mode_;
        snap.low_cmd_type   = low_cmd_type_;
        snap.low_seq        = low_seq_;
        snap.low_received   = low_received_;
        snap.low_cmd_motors = low_cmd_motors_;
        return snap;
    }

    void LocomotionController::set_head_command(double pitch, double yaw) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        head_pitch_ = pitch;
        head_yaw_   = yaw;
    }

    void LocomotionController::request_mode_change(int mode) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        requested_mode_ = mode;
        ++mode_seq_;
    }

    void LocomotionController::set_low_cmd(int cmd_type, std::vector<message::MotorCmdData> motors) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        low_cmd_type_ = cmd_type;
        ++low_seq_;
        low_received_   = std::chrono::steady_clock::now();
        low_cmd_motors_ = std::move(motors);
    }

    void LocomotionController::step(const mjModel* m, mjData* d) {
        ensure_initialized(m);
        const CommandSnapshot snap = snapshot_command();

        if (snap.mode_seq != last_mode_seq_) {
            last_mode_seq_ = snap.mode_seq;
            handle_mode_request(snap.requested_mode, d);
        }

        update_fall_detection(m, d);

        switch (state_) {
            case State::Damping: step_damping(d); break;
            case State::Prepare: step_prepare(m, d, snap); break;
            case State::Custom: step_custom(m, d, snap); break;
        }
    }

    void LocomotionController::handle_mode_request(int requested_mode, mjData* d) {
        int effective = requested_mode;
        switch (requested_mode) {
            case booster::DAMPING:
            case booster::PREPARE:
            case booster::CUSTOM: break;
            case booster::WALKING:
            case booster::SOCCER:
                // Wire-compatible, but the walking policy lives on the NUbots_K1 side now
                // (rt/joint_ctrl in CUSTOM mode). Hold the ready pose instead.
                std::fprintf(stderr,
                             "LocomotionController: ChangeMode(%d) requested, but locomotion policies "
                             "live in NUbots_K1 now; holding PREPARE (use CUSTOM + rt/joint_ctrl)\n",
                             requested_mode);
                effective = booster::PREPARE;
                break;
            default:
                std::fprintf(stderr, "LocomotionController: ignoring unknown ChangeMode value %d\n", requested_mode);
                return;
        }

        mode_.store(effective, std::memory_order_relaxed);
        switch (effective) {
            case booster::DAMPING: state_ = State::Damping; break;
            case booster::PREPARE:
                state_              = State::Prepare;
                prepare_start_pose_ = current_q(d);
                prepare_start_time_ = d->time;
                break;
            case booster::CUSTOM:
                state_                  = State::Custom;
                custom_parallel_warned_ = false;
                // PD-hold the pose we entered CUSTOM in until the first LowCmd arrives;
                // freezing raw torques instead lets the robot slump if the client dies
                // (or never manages to stream) after switching modes. Drop any LowCmd left
                // over from a previous CUSTOM session for the same reason.
                custom_entry_pose_   = current_q(d);
                custom_low_cmd_seen_ = false;
                custom_entry_step_   = true;
                {
                    std::lock_guard<std::mutex> lock(cmd_mutex_);
                    low_cmd_motors_.clear();
                }
                break;
            default: break;  // unreachable
        }
    }

    void LocomotionController::step_damping(mjData* d) {
        for (int act : map_->act_id) {
            d->ctrl[act] = 0.0;
        }
    }

    void LocomotionController::step_prepare(const mjModel* m, mjData* d, const CommandSnapshot& snap) {
        const double frac = std::clamp((d->time - prepare_start_time_) / prepare_blend_time_, 0.0, 1.0);
        const double s    = smoothstep(frac);

        std::array<double, JOINT_COUNT> q_ref{};
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            q_ref[i] = lerp(prepare_start_pose_[i], ready_pose_[i], s);
        }
        apply_head_target(q_ref, snap.head_pitch, snap.head_yaw);
        pd_.apply(m, d, *map_, q_ref);
    }

    void LocomotionController::step_custom(const mjModel* m, mjData* d, const CommandSnapshot& snap) {
        // Until the first LowCmd of this CUSTOM session arrives, PD-hold the entry pose
        // (a dead or struggling client must not leave the robot on frozen torques). The
        // entry step's snapshot predates the mailbox clear, so it is skipped explicitly.
        if (custom_entry_step_) {
            custom_entry_step_ = false;
            pd_.apply(m, d, *map_, custom_entry_pose_);
            return;
        }
        if (!custom_low_cmd_seen_) {
            if (!snap.low_cmd_motors.empty()) {
                custom_low_cmd_seen_ = true;
            }
            else {
                pd_.apply(m, d, *map_, custom_entry_pose_);
                return;
            }
        }

        if (snap.low_cmd_type == 0) {  // PARALLEL: unsupported, hold last ctrl
            if (!custom_parallel_warned_) {
                std::fprintf(stderr,
                             "LocomotionController: CUSTOM received cmd_type=PARALLEL, which is not "
                             "supported; holding last ctrl\n");
                custom_parallel_warned_ = true;
            }
            return;
        }

        if (low_cmd_timeout_ > 0.
            && std::chrono::duration<double>(std::chrono::steady_clock::now() - snap.low_received).count()
                   > low_cmd_timeout_) {
            // A disconnected controller holds the current pose rather than pursuing stale targets.
            if (!stale_hold_) {
                stale_pose_ = current_q(d);
                stale_hold_ = true;
            }
            pd_.apply(m, d, *map_, stale_pose_);
            return;
        }

        stale_hold_         = false;
        const auto& motors  = snap.low_cmd_motors;
        const std::size_t n = std::min(motors.size(), JOINT_COUNT);
        for (std::size_t i = 0; i < n; ++i) {
            const auto& mc  = motors[i];
            const int act   = map_->act_id[i];
            const double q  = d->qpos[map_->qpos_adr[i]];
            const double dq = d->qvel[map_->dof_adr[i]];
            double tau      = mc.kp * (mc.q - q) + mc.kd * (mc.dq - dq) + mc.tau;
            if (m->actuator_forcelimited[act] != 0) {
                tau = std::clamp(tau, m->actuator_forcerange[2 * act], m->actuator_forcerange[2 * act + 1]);
            }
            d->ctrl[act] = tau;
        }
        if (n && snap.low_seq != applied_low_seq_) {
            applied_low_seq_ = snap.low_seq;
            latency_ns_.fetch_add(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()
                                                                                       - snap.low_received)
                                      .count());
            latency_count_.fetch_add(1);
        }

        // Joints beyond `n` (an undersized LowCmd) are left at their previous
        // ctrl value -- the same "hold" behaviour as a PARALLEL command.
    }

    void LocomotionController::update_fall_detection(const mjModel* /*m*/, mjData* d) {
        const double tilt = base_tilt(d, *map_);
        double gyro_mag   = 0.0;
        if (map_->sens_gyro >= 0) {
            const double* g = &d->sensordata[map_->sens_gyro];
            gyro_mag        = std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
        }

        // Height criterion catches collapsed-but-trunk-upright poses (kneeling crumples
        // after a failed get-up) that tilt alone reads as IS_READY -- which starves the
        // NUbots GetUp retry loop forever.
        const double base_z = d->qpos[map_->root_qpos_adr + 2];

        int computed = booster::IS_READY;
        if (tilt > fallen_tilt_ || base_z < fallen_height_) {
            computed = booster::HAS_FALLEN;
        }
        else if (tilt > falling_tilt_ && gyro_mag > falling_gyro_) {
            computed = booster::IS_FALLING;
        }

        fall_state_.store(computed, std::memory_order_relaxed);
    }

}  // namespace k1sim::module
