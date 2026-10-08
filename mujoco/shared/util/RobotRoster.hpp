#ifndef K1SIM_SHARED_UTIL_ROBOTROSTER_HPP
#define K1SIM_SHARED_UTIL_ROBOTROSTER_HPP

#include <stdexcept>
#include <vector>

#include "shared/sim/MatchConfig.hpp"
#include "shared/util/Config.hpp"

namespace k1sim::config {

    // Read once per process, after parse_cli(), before Startup reactions run.
    inline const std::vector<RobotIdentity>& robot_roster() {
        static const std::vector<RobotIdentity> roster = [] {
            const int count = cli().game > 0 ? 2 * cli().game : cli().robots;
            if (!cli().match_config.empty()) {
                const auto path = std::filesystem::path(cli().match_config);
                const auto match =
                    load_match_config(YAML::LoadFile((path.is_absolute() ? path : config_dir() / path).string()));
                if (cli().game == 0 || static_cast<int>(match.robots.size()) != count) {
                    throw std::invalid_argument("--match roster size must agree with --game");
                }
                return match.robots;
            }
            const int first_domain = load("dds.yaml")["domain"].as<int>(0);
            if (count > 1 && (first_domain < 0 || first_domain > 64 - count)) {
                throw std::invalid_argument("multi-robot DDS domains must fit in [0, 63]; use --match to override");
            }
            std::vector<RobotIdentity> result;
            const int team_size = cli().game > 0 ? cli().game : 11;
            for (int i = 0; i < count; ++i) {
                result.push_back({i + 1, i, 125 + i / team_size, i % team_size + 1, first_domain + i});
            }
            return result;
        }();
        return roster;
    }

}  // namespace k1sim::config

#endif  // K1SIM_SHARED_UTIL_ROBOTROSTER_HPP
