#ifndef K1SIM_SHARED_SIM_MATCHCONFIG_HPP
#define K1SIM_SHARED_SIM_MATCHCONFIG_HPP

#include <array>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "shared/sim/RobotIdentity.hpp"

namespace k1sim {

    struct TeamIdentity {
        int team_id                     = 0;
        bool initial_attacks_positive_x = true;
    };

    // A roster follows --game's scene order: team 0's slots, then team 1's slots.
    // --match selects this schema for Simulation, Locomotion and SdkBridge.
    struct MatchConfig {
        int schema_version = 1;
        std::array<TeamIdentity, 2> teams{};
        std::vector<RobotIdentity> robots;

        void validate() const {
            if (schema_version != 1) {
                throw std::invalid_argument("unsupported match schema_version");
            }
            if (robots.empty() || robots.size() > 22 || robots.size() % 2 != 0) {
                throw std::invalid_argument("match requires two equally sized teams of 1..11 robots");
            }
            for (const auto& team : teams) {
                if (team.team_id < 1 || team.team_id > 255) {
                    throw std::invalid_argument("team_id must be in [1, 255]");
                }
            }
            if (teams[0].team_id == teams[1].team_id
                || teams[0].initial_attacks_positive_x == teams[1].initial_attacks_positive_x) {
                throw std::invalid_argument("teams need distinct IDs and opposite attack directions");
            }
            std::set<int> domains;
            std::set<std::pair<int, int>> players;
            const int team_size = static_cast<int>(robots.size() / 2);
            for (int i = 0; i < static_cast<int>(robots.size()); ++i) {
                const auto& robot = robots[i];
                if (robot.model_index != i || robot.robot_id != i + 1) {
                    throw std::invalid_argument("robot_id must equal model_index + 1 in scene order");
                }
                if (robot.team_id != teams[i / team_size].team_id || robot.player_id < 1 || robot.player_id > 11
                    || !players.emplace(robot.team_id, robot.player_id).second) {
                    throw std::invalid_argument("invalid team assignment or duplicate/out-of-range player_id");
                }
                // Local simulator convention; reserve a small domain range for match instances.
                if (robot.dds_domain < 0 || robot.dds_domain > 63 || !domains.insert(robot.dds_domain).second) {
                    throw std::invalid_argument("dds_domain must be unique within the match and in [0, 63]");
                }
            }
        }
    };

    inline MatchConfig load_match_config(const YAML::Node& root) {
        MatchConfig cfg;
        cfg.schema_version = root["schema_version"].as<int>();
        const auto teams   = root["teams"];
        const auto robots  = root["robots"];
        if (!teams.IsSequence() || teams.size() != 2 || !robots.IsSequence()) {
            throw std::invalid_argument("match needs exactly two teams and a robot sequence");
        }
        for (std::size_t i = 0; i < cfg.teams.size(); ++i) {
            cfg.teams[i].team_id                    = teams[i]["team_id"].as<int>();
            cfg.teams[i].initial_attacks_positive_x = teams[i]["initial_attacks_positive_x"].as<bool>();
        }
        for (const auto& node : robots) {
            RobotIdentity robot;
            robot.model_index = static_cast<int>(cfg.robots.size());
            robot.robot_id    = node["robot_id"].as<int>();
            robot.team_id     = node["team_id"].as<int>();
            robot.player_id   = node["player_id"].as<int>();
            robot.dds_domain  = node["dds_domain"].as<int>();
            cfg.robots.push_back(robot);
        }
        cfg.validate();
        return cfg;
    }

}  // namespace k1sim

#endif  // K1SIM_SHARED_SIM_MATCHCONFIG_HPP
