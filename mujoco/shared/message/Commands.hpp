#ifndef K1SIM_SHARED_MESSAGE_COMMANDS_HPP
#define K1SIM_SHARED_MESSAGE_COMMANDS_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "shared/sim/StepController.hpp"

// NUClear messages emitted by module::SdkBridge when Booster SDK RPCs / topics
// arrive, consumed by module::Locomotion. One struct per LocoApi call NUbots uses.

namespace k1sim::message {

    struct WalkCommand {  // ApiId::MOVE {"vx","vy","vyaw"} — body-frame velocities
        double vx    = 0.0;
        double vy    = 0.0;
        double vyaw  = 0.0;
        int robot_id = 1;  // in-process routing; Booster wire format is unchanged
    };

    struct HeadCommand {  // ApiId::ROTATE_HEAD {"pitch","yaw"} — pitch down-positive
        double pitch = 0.0;
        double yaw   = 0.0;
        int robot_id = 1;  // in-process routing; Booster wire format is unchanged
    };

    struct ModeChangeRequest {  // ApiId::CHANGE_MODE — booster::RobotMode value
        int mode     = 0;
        int robot_id = 1;  // in-process routing; Booster wire format is unchanged
    };

    struct GetUpRequest {     // ApiId::GET_UP / GET_UP_WITH_MODE — mode to enter afterwards
        int target_mode = 4;  // booster::RobotMode::SOCCER (what NUbots requests)
        int robot_id    = 1;  // in-process routing; Booster wire format is unchanged
    };

    struct LieDownRequest {
        int robot_id = 1;
    };

    struct VisualKickRequest {  // ApiId::VISUAL_KICK
        bool start   = true;
        int version  = 1;
        int robot_id = 1;  // in-process routing; Booster wire format is unchanged
    };

    struct MotorCmdData {  // one LowCmd MotorCmd (serial order)
        uint8_t mode = 0;
        float q = 0, dq = 0, tau = 0, kp = 0, kd = 0, weight = 0;
    };

    struct LowCmdMessage {  // rt/joint_ctrl, only honoured in RobotMode::CUSTOM
        int cmd_type = 1;   // 0 = PARALLEL (logged + ignored), 1 = SERIAL
        std::vector<MotorCmdData> motors;
        int robot_id = 1;  // in-process routing; Booster wire format is unchanged
    };

    // rt/nusim/ball_command (NUSim test control, see shared/k1/NUSimApi.hpp): place the scene ball
    // and set its velocity. Consumed by module::Supervisor, which applies it under the sim mutex.
    struct BallCommand {
        enum class Frame { WORLD, ROBOT };
        Frame frame = Frame::WORLD;
        std::array<double, 3> position{};          // ball centre; z < 0 rests it on the floor
        std::array<double, 3> velocity{};          // centre velocity (m/s)
        std::array<double, 3> angular_velocity{};  // spin (rad/s), ignored when rolling
        bool rolling = false;                      // derive spin for rolling without slipping
        // Trunk body of the robot whose domain the command arrived on; Frame::ROBOT is relative to it
        std::string robot_body = "Trunk";
    };

    // Emitted once by module::Locomotion at startup; consumed by module::Simulation,
    // whose physics thread drives controller->step() (see StepController).
    struct ControllerHandle {
        StepController* controller = nullptr;
        int robot_id               = 1;
        std::shared_ptr<StepController> owner;
    };

    // Backspace resets the whole world and every controller.
    struct SimResetRequest {};

    // Internal per-robot reset: restore its startup pose and clear its controller.
    // The ball, global clock and other robots are untouched.
    struct RobotResetRequest {
        int robot_id = 1;
    };

}  // namespace k1sim::message

#endif  // K1SIM_SHARED_MESSAGE_COMMANDS_HPP
