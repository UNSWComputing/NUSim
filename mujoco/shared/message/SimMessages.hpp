#ifndef K1SIM_SHARED_MESSAGE_SIMMESSAGES_HPP
#define K1SIM_SHARED_MESSAGE_SIMMESSAGES_HPP

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mujoco/mujoco.h>
#include <mutex>
#include <vector>

#include "shared/k1/JointIndex.hpp"
#include "shared/sim/RobotIdentity.hpp"

// NUClear messages exchanged between the sim's own modules (in-process only —
// nothing here goes over DDS; the SdkBridge translates to the Booster wire types).

namespace k1sim::message {

    struct JointState {
        double q   = 0.0;  // rad
        double dq  = 0.0;  // rad/s
        double ddq = 0.0;  // rad/s^2
        double tau = 0.0;  // N·m (actuator force)
    };

    struct ImuData {
        std::array<double, 4> quat{1, 0, 0, 0};  // w,x,y,z imu->world
        std::array<double, 3> rpy{};             // roll, pitch, yaw (rad)
        std::array<double, 3> gyro{};            // rad/s, body frame
        std::array<double, 3> acc{};             // m/s^2, body frame, includes gravity
    };

    struct BaseState {
        double x = 0.0, y = 0.0, z = 0.0;        // world frame
        std::array<double, 4> quat{1, 0, 0, 0};  // w,x,y,z
        std::array<double, 3> lin_vel{};         // world frame
        std::array<double, 3> ang_vel{};         // world frame
    };

    // The real robot's head frame (0.08 m above Head_pitch) in the yaw-only base footprint
    // frame (see shared/sim/HeadPose.hpp).
    struct HeadPose {
        bool valid = false;                      // false if the model has no Head_2 body
        std::array<double, 3> position{};        // m
        std::array<double, 4> quat{1, 0, 0, 0};  // w,x,y,z
    };

    // The scene ball (body "ball", sphere geom "ball"), for NUSim ground truth (rt/nusim/gt/ball).
    struct BallState {
        bool valid = false;                  // false if the scene has no ball
        std::array<double, 3> position{};    // geom centre, world frame (m)
        std::array<double, 3> lin_vel{};     // geom centre velocity, world frame (m/s)
        std::array<double, 3> ang_vel{};     // world frame (rad/s)
    };

    // Emitted by the physics thread at the LowState cadence (every N steps, 50 Hz).
    struct SimStateUpdate {
        double sim_time     = 0.0;
        int64_t wall_time_ns = 0;  // system clock (ns since epoch) when the snapshot was taken
        uint64_t step_count = 0;
        std::array<JointState, JOINT_COUNT> joints{};  // JointIndexK1 order
        ImuData imu{};
        BaseState base{};
        HeadPose head{};
        BallState ball{};
        int mode            = 0;  // booster::RobotMode value
        int fall_state      = 0;  // booster::FallState value
        bool getting_up     = false;
        double measured_rtf = 0.0;
        RobotIdentity identity{1, 0, 125, 1, 0};
        uint64_t reset_count = 0;  // per-robot resets, including full-world resets
    };

    // Captured under one physics lock. Used by SdkBridge so NUClear's per-type
    // cache cannot substitute another robot's latest state for an earlier one.
    struct RobotStatesUpdate {
        std::vector<SimStateUpdate> robots;
        bool ball_valid = false;
        std::array<double, 3> ball_centre{}, ball_velocity{}, ball_angular_velocity{};
        uint64_t reset_generation     = 0;
        uint64_t sample_sequence      = 0;
        uint64_t capture_time_unix_ns = 0;
        std::array<char, 36> session_id{};
    };

    // Emitted once by module::Simulation after the model is loaded. The mutex guards
    // mjData; mjModel is immutable after load. Pointers are valid Startup->Shutdown.
    struct SimHandles {
        const mjModel* model              = nullptr;
        mjData* data                      = nullptr;
        std::mutex* mutex                 = nullptr;
        std::atomic<double>* measured_rtf = nullptr;
        // Caller holds mutex. Clear the relocated robot's controller and signal its reset.
        std::function<void(int)> placed_robot;
        // Acquires mutex internally; call before acquiring the physics lock.
        std::function<void()> reset_world;
    };

}  // namespace k1sim::message

#endif  // K1SIM_SHARED_MESSAGE_SIMMESSAGES_HPP
