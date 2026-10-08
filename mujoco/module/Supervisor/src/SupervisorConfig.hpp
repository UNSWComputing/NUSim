#ifndef K1SIM_MODULE_SUPERVISOR_SUPERVISORCONFIG_HPP
#define K1SIM_MODULE_SUPERVISOR_SUPERVISORCONFIG_HPP

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

// config/supervisor.yaml schema, parsed independently of NUClear so
// test_supervisor.cpp can build a SupervisorConfig without touching the
// Reactor — mirrors how module::Simulation's build_sim_config() reads
// simulation.yaml/gains.yaml synchronously at construction time (this port
// doesn't use NUClear's on<Configuration> hot-reload extension anywhere).
namespace k1sim::module::supervisor {

    // A flat-ground standing pose: world (x, y, z) plus yaw about +Z (radians).
    struct Pose3 {
        double x   = 0.0;
        double y   = 0.0;
        double z   = 0.0;
        double yaw = 0.0;
    };

    struct BallConfig {
        bool enabled     = true;
        std::string body = "ball";
        std::string geom = "ball";
        double x         = 0.0;
        double y         = 0.0;
        // z < 0 means "rest the ball on the floor at its own geom radius";
        // only set this explicitly to override that.
        double z = -1.0;
    };

    // One managed robot body. team_id/team_index resolve which of the packet's
    // two Team entries governs this robot: if team_id >= 0 it is matched against
    // GameControllerPacket::Team::team_id (like NUbots' own GameController
    // resolves "our" team); otherwise team_index (0 or 1) just picks a packet
    // slot positionally, which is all local/single-robot testing needs before a
    // real competition team_id is configured. player_id is 1-based, indexing
    // that team's players[] array (RoboCup convention: player 1..N).
    struct RobotConfig {
        std::string body;
        int robot_id   = 1;
        int team_id    = -1;
        int team_index = 0;
        int player_id  = 1;
        Pose3 home_pose;     // where an unpenalised robot belongs (its own half)
        Pose3 penalty_pose;  // where a penalised robot is moved (side line)
    };

    struct SupervisorConfig {
        bool enabled = true;
        int gc_port  = 3838;

        // Extra ball-centre resets beyond the core kickoff rule (entering READY,
        // or entering PLAYING) — see SupervisorLogic::process. All optional.
        bool reset_ball_on_finished    = true;
        bool reset_ball_on_goal        = true;
        bool reset_ball_on_half_change = true;

        BallConfig ball;
        std::vector<RobotConfig> robots;
    };

    inline Pose3 load_pose(const YAML::Node& node, const Pose3& fallback = Pose3{}) {
        if (!node) {
            return fallback;
        }
        Pose3 pose;
        pose.x   = node["x"].as<double>(fallback.x);
        pose.y   = node["y"].as<double>(fallback.y);
        pose.z   = node["z"].as<double>(fallback.z);
        pose.yaw = node["yaw"].as<double>(fallback.yaw);
        return pose;
    }

    inline SupervisorConfig load_config(const YAML::Node& root) {
        SupervisorConfig cfg;
        cfg.enabled = root["enabled"].as<bool>(cfg.enabled);
        cfg.gc_port = root["gc_port"].as<int>(cfg.gc_port);

        if (const auto& reset = root["reset_ball_on"]) {
            cfg.reset_ball_on_finished    = reset["finished"].as<bool>(cfg.reset_ball_on_finished);
            cfg.reset_ball_on_goal        = reset["goal"].as<bool>(cfg.reset_ball_on_goal);
            cfg.reset_ball_on_half_change = reset["half_change"].as<bool>(cfg.reset_ball_on_half_change);
        }

        if (const auto& ball = root["ball"]) {
            cfg.ball.enabled = ball["enabled"].as<bool>(cfg.ball.enabled);
            cfg.ball.body    = ball["body"].as<std::string>(cfg.ball.body);
            cfg.ball.geom    = ball["geom"].as<std::string>(cfg.ball.geom);
            cfg.ball.x       = ball["x"].as<double>(cfg.ball.x);
            cfg.ball.y       = ball["y"].as<double>(cfg.ball.y);
            cfg.ball.z       = ball["z"].as<double>(cfg.ball.z);
        }

        if (const auto& robots = root["robots"]) {
            for (const auto& r : robots) {
                RobotConfig rc;
                rc.robot_id     = r["robot_id"].as<int>(1);
                rc.body         = r["body"].as<std::string>();
                rc.team_id      = r["team_id"].as<int>(rc.team_id);
                rc.team_index   = r["team_index"].as<int>(rc.team_index);
                rc.player_id    = r["player_id"].as<int>(rc.player_id);
                rc.home_pose    = load_pose(r["home_pose"]);
                rc.penalty_pose = load_pose(r["penalty_pose"]);
                cfg.robots.push_back(rc);
            }
        }

        return cfg;
    }

}  // namespace k1sim::module::supervisor

#endif  // K1SIM_MODULE_SUPERVISOR_SUPERVISORCONFIG_HPP
