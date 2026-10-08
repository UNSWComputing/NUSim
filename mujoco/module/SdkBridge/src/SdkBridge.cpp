#include "module/SdkBridge/src/SdkBridge.hpp"

#include <cstdlib>
#include <string>

#include "shared/message/Commands.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/Config.hpp"
#include "shared/util/RobotRoster.hpp"

namespace k1sim::module {

    namespace {
        bool env_flag_set(const char* name) {
            const char* value = std::getenv(name);
            return value != nullptr && std::string(value) != "0" && std::string(value) != "";
        }
    }  // namespace

    SdkBridge::SdkBridge(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Startup>().then([this] {
            auto cfg = config::load("dds.yaml");

            const bool udp_only              = cfg["udp_only"].as<bool>(false) || env_flag_set("K1_DDS_UDP_ONLY");
            const double battery_soc         = cfg["battery_soc"].as<double>(100.0);
            const int64_t unknown_api_status = cfg["unknown_api_status"].as<int64_t>(0);

            const auto odometry = sdkbridge::OdometryModel::load_config(config::load("odometry.yaml"));

            std::lock_guard<std::mutex> lock(connections_mutex_);
            for (const auto& robot : config::robot_roster()) {
                // Each robot gets its own odometry estimator (own bias, pose and noise stream); a fixed
                // seed is offset per robot so they do not all draw the same noise.
                auto robot_odometry = odometry;
                if (robot_odometry.seed != 0) {
                    robot_odometry.seed += static_cast<uint64_t>(robot.robot_id);
                }

                RobotConnection connection;
                connection.robot_id = robot.robot_id;
                connection.dds      = std::make_unique<sdkbridge::DdsParticipant>(robot.dds_domain, udp_only);
                connection.publisher =
                    std::make_unique<sdkbridge::StatePublisher>(*connection.dds, battery_soc, robot_odometry);
                connection.rpc =
                    std::make_unique<sdkbridge::RpcServer>(*connection.dds, *this, unknown_api_status, robot.robot_id);
                connection.ground_truth        = std::make_unique<sdkbridge::GroundTruthPublisher>(*connection.dds);
                connection.ball_command_reader =
                    std::make_unique<sdkbridge::BallCommandReader>(*connection.dds, *this, robot);
                connections_.push_back(std::move(connection));
                log<NUClear::LogLevel::INFO>("SdkBridge ready — robot",
                                             robot.robot_id,
                                             "DDS domain",
                                             robot.dds_domain,
                                             udp_only ? "UDP-only" : "UDP+SHM",
                                             ", odometry",
                                             odometry.enabled ? "with the config/odometry.yaml error model"
                                                              : "ground truth");
            }
        });

        on<Trigger<message::RobotStatesUpdate>>().then([this](const message::RobotStatesUpdate& update) {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            for (const auto& state : update.robots) {
                for (auto& connection : connections_) {
                    if (connection.robot_id == state.identity.robot_id) {
                        // NUClear tasks can acquire this lock out of capture order. Never
                        // republish an older sample, especially one queued before a reset.
                        if (connection.seen_state
                            && (state.reset_count < connection.last_reset_count
                                || (state.reset_count == connection.last_reset_count
                                    && state.step_count <= connection.last_step_count))) {
                            break;
                        }
                        connection.seen_state       = true;
                        connection.last_reset_count = state.reset_count;
                        connection.last_step_count  = state.step_count;
                        connection.rpc->set_current_mode(state.mode);
                        connection.publisher->publish(state);
                        connection.ground_truth->publish(state);
                        break;
                    }
                }
            }
        });

        on<Every<1, std::chrono::seconds>>().then([this] {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            for (auto& connection : connections_) {
                connection.publisher->publish_battery();
            }
        });

        on<Shutdown>().then([this] {
            log<NUClear::LogLevel::INFO>("SdkBridge shutting down");
            std::lock_guard<std::mutex> lock(connections_mutex_);
            connections_.clear();
        });
    }

}  // namespace k1sim::module
