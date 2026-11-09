#ifndef K1SIM_SHARED_MESSAGE_SIMMESSAGES_HPP
#define K1SIM_SHARED_MESSAGE_SIMMESSAGES_HPP

#include <array>
#include <atomic>
#include <cstdint>
#include <mujoco/mujoco.h>
#include <mutex>
#include <string>
#include <vector>

#include "shared/k1/JointIndex.hpp"

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
        std::array<double, 4> quat{1, 0, 0, 0};  // w,x,y,z world->imu
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

    // Emitted by the physics thread at the LowState cadence (every N steps, 50 Hz).
    struct SimStateUpdate {
        double sim_time     = 0.0;
        uint64_t step_count = 0;
        std::array<JointState, JOINT_COUNT> joints{};  // JointIndexK1 order
        ImuData imu{};
        BaseState base{};
        HeadPose head{};
        int mode            = 0;  // booster::RobotMode value
        int fall_state      = 0;  // booster::FallState value
        bool getting_up     = false;
        double measured_rtf = 0.0;
    };

    // Emitted once by module::Simulation after the model is loaded. The mutex guards
    // mjData; mjModel is immutable after load. Pointers are valid Startup->Shutdown.
    // What the referee (RoboCup GameController) last announced. Emitted by module::Supervisor for every packet it
    // accepts; shown on the browser viewer's scoreboard.
    struct GameState {
        std::string left;        // name of the first team in the packet
        std::string right;       // name of the second team in the packet
        int score_left     = 0;
        int score_right    = 0;
        int secs_remaining = 0;  // seconds left in the current half
        std::string mode;        // words separated by " | ", e.g. "first half | PLAYING"

        std::string state;        // INITIAL, READY, SET, PLAYING or FINISHED
        bool first_half    = true;
        int secondary_time = 0;   // seconds of the secondary clock (ready time, free kick, ...)
        int colour_left    = 0;   // field player colours, GameController numbering (0 blue, 1 red, ...)
        int colour_right   = 1;
        int kicking_side   = -1;  // 0 = left team, 1 = right team, -1 = nobody

        struct Robot {            // one simulated robot the GameController governs
            std::string body;     // its root body in the model
            int side   = 0;       // 0 = left team, 1 = right team
            int player = 1;       // player number, starting at 1
            std::string penalty;  // empty while in play, else the reason (e.g. "PLAYER_PUSHING")
            int penalty_secs = 0; // estimated seconds until the penalty ends
        };
        std::vector<Robot> robots;
    };

    // A referee decision, from the browser viewer's referee panel to module::Referee.
    struct RefereeCommand {
        std::string action;  // kickoff, set, play, stop, resume, goal, penalize, unpenalize, setplay, timeout, ...
        std::string side;    // "home", "away" or empty
        int player = 0;      // player number for penalize / unpenalize
        std::string arg;     // penalty call or set play, e.g. "pushing" or "corner_kick"
    };

    // What the built-in referee currently allows, and the outcome of the last command.
    struct RefereeStatus {
        std::vector<std::string> legal;  // e.g. "kickoff:home", "set", "goal:away"
        int result_seq = 0;              // counts the commands handled so far
        bool result_ok = true;
        std::string result_text;
    };

    struct SimHandles {
        const mjModel* model              = nullptr;
        mjData* data                      = nullptr;
        std::mutex* mutex                 = nullptr;
        std::atomic<double>* measured_rtf = nullptr;
    };

}  // namespace k1sim::message

#endif  // K1SIM_SHARED_MESSAGE_SIMMESSAGES_HPP
