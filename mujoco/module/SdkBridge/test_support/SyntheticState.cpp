#include "module/SdkBridge/test_support/SyntheticState.hpp"

#include <cmath>

#include "shared/k1/BoosterApi.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/message/Commands.hpp"
#include "shared/message/SimMessages.hpp"

namespace k1sim::module::sdkbridge::test_support {

    namespace {
        constexpr double kDt = 1.0 / 50.0;
    }

    SyntheticState::SyntheticState(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Startup>().then(
            [this] { log<NUClear::LogLevel::INFO>("SyntheticState ready (test-only SimStateUpdate source, 50 Hz)"); });

        on<Trigger<k1sim::message::ModeChangeRequest>>().then([this](const k1sim::message::ModeChangeRequest& msg) {
            std::scoped_lock lock(mutex_);
            mode_ = msg.mode;
        });

        on<Trigger<k1sim::message::WalkCommand>>().then([this](const k1sim::message::WalkCommand& msg) {
            std::scoped_lock lock(mutex_);
            vx_   = msg.vx;
            vy_   = msg.vy;
            vyaw_ = msg.vyaw;
        });

        on<Trigger<k1sim::message::HeadCommand>>().then([this](const k1sim::message::HeadCommand& msg) {
            std::scoped_lock lock(mutex_);
            head_pitch_ = msg.pitch;
            head_yaw_   = msg.yaw;
        });

        on<Trigger<k1sim::message::GetUpRequest>>().then([this](const k1sim::message::GetUpRequest& msg) {
            std::scoped_lock lock(mutex_);
            fall_state_ = k1sim::booster::IS_READY;
            mode_       = msg.target_mode;
        });

        on<Trigger<k1sim::message::LieDownRequest>>().then([this] {
            std::scoped_lock lock(mutex_);
            fall_state_ = k1sim::booster::HAS_FALLEN;
        });

        on<Trigger<k1sim::message::VisualKickRequest>>().then(
            [this](const k1sim::message::VisualKickRequest& msg) { (void) msg; });

        on<Every<20, std::chrono::milliseconds>>().then([this] {
            auto update = std::make_unique<k1sim::message::SimStateUpdate>();

            std::scoped_lock lock(mutex_);
            sim_time_ += kDt;
            step_count_ += 1;

            // Planar odometry integration (body-frame vx/vy rotated by current yaw).
            const double cos_yaw = std::cos(yaw_);
            const double sin_yaw = std::sin(yaw_);
            x_ += (vx_ * cos_yaw - vy_ * sin_yaw) * kDt;
            y_ += (vx_ * sin_yaw + vy_ * cos_yaw) * kDt;
            yaw_ += vyaw_ * kDt;

            update->sim_time   = sim_time_;
            update->step_count = step_count_;

            // Plausible standing IMU: level attitude, near-zero gyro except the
            // commanded yaw rate, gravity on the accelerometer.
            update->imu.rpy  = {0.0, 0.0, yaw_};
            update->imu.gyro = {0.0, 0.0, vyaw_};
            update->imu.acc  = {0.0, 0.0, 9.81};

            for (std::size_t i = 0; i < k1sim::JOINT_COUNT; ++i) {
                update->joints[i] = {};
            }
            update->joints[k1sim::HeadYaw].q   = head_yaw_;
            update->joints[k1sim::HeadPitch].q = head_pitch_;

            update->base.x    = x_;
            update->base.y    = y_;
            update->base.z    = 0.53;
            const double half = yaw_ * 0.5;
            update->base.quat = {std::cos(half), 0.0, 0.0, std::sin(half)};
            // World-frame velocity of the commanded body-frame walk
            update->base.lin_vel = {vx_ * cos_yaw - vy_ * sin_yaw, vx_ * sin_yaw + vy_ * cos_yaw, 0.0};
            update->base.ang_vel = {0.0, 0.0, vyaw_};

            // Standing head frame in the footprint frame: 0.33 m above the base (the Head_pitch
            // joint 0.248 m above the Trunk plus the 0.08 m head frame offset, see
            // shared/sim/HeadPose.hpp), turned by the commanded head yaw then pitch.
            const double cy = std::cos(head_yaw_ * 0.5), sy = std::sin(head_yaw_ * 0.5);
            const double cp = std::cos(head_pitch_ * 0.5), sp = std::sin(head_pitch_ * 0.5);
            update->head.valid    = true;
            update->head.position = {0.0, 0.0, update->base.z + 0.33};
            update->head.quat     = {cy * cp, -sy * sp, cy * sp, sy * cp};

            update->mode         = mode_;
            update->fall_state   = fall_state_;
            update->getting_up   = false;
            update->measured_rtf = 1.0;

            auto batch = std::make_unique<k1sim::message::RobotStatesUpdate>();
            batch->robots.push_back(*update);
            emit(batch);
        });
    }

}  // namespace k1sim::module::sdkbridge::test_support
