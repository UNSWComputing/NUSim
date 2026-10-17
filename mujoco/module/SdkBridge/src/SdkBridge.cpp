#include "module/SdkBridge/src/SdkBridge.hpp"

#include <cstdlib>
#include <string>

#include "WorldSnapshotPubSubTypes.h"
#include "shared/message/Commands.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/sim/GroundTruthContract.hpp"
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
            const bool local_only            = cfg["local_only"].as<bool>(false);
            const double battery_soc         = cfg["battery_soc"].as<double>(100.0);
            const int64_t unknown_api_status = cfg["unknown_api_status"].as<int64_t>(0);

            const auto odometry = sdkbridge::OdometryModel::load_config(config::load("odometry.yaml"));

            std::lock_guard<std::mutex> lock(connections_mutex_);
            if (!cli().match_config.empty()) {
                const auto path = std::filesystem::path(cli().match_config);
                teams_          = load_match_config(
                                      YAML::LoadFile((path.is_absolute() ? path : config::config_dir() / path).string()))
                                      .teams;
            }
            for (const auto& robot : config::robot_roster()) {
                // Each robot gets its own odometry estimator (own bias, pose and noise stream); a fixed
                // seed is offset per robot so they do not all draw the same noise.
                auto robot_odometry = odometry;
                if (robot_odometry.seed != 0) {
                    robot_odometry.seed += static_cast<uint64_t>(robot.robot_id);
                }

                RobotConnection connection;
                connection.robot_id = robot.robot_id;
                connection.dds = std::make_unique<sdkbridge::DdsParticipant>(robot.dds_domain, udp_only, local_only);
                connection.truth_writer =
                    connection.dds->create_writer<nusim_msgs::msg::dds_::WorldSnapshot_PubSubType>(
                        ground_truth::TOPIC_WORLD_SNAPSHOT,
                        sdkbridge::DdsParticipant::state_writer_qos());
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
                                             local_only ? "loopback UDP" : (udp_only ? "UDP-only" : "UDP+SHM"),
                                             ", odometry",
                                             odometry.enabled ? "with the config/odometry.yaml error model"
                                                              : "ground truth");
            }
        });

        on<Trigger<message::RobotStatesUpdate>>().then([this](const message::RobotStatesUpdate& update) {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            if (update.robots.empty())
                return;
            nusim_msgs::msg::dds_::WorldSnapshot_ world;
            world.stamp().schema_version(ground_truth::SCHEMA_VERSION);
            world.stamp().session_id(update.session_id);
            world.stamp().reset_generation(update.reset_generation);
            world.stamp().sample_sequence(update.sample_sequence);
            world.stamp().step_count(update.robots.front().step_count);
            world.stamp().sim_time(update.robots.front().sim_time);
            world.stamp().capture_time_unix_ns(update.capture_time_unix_ns);
            for (std::size_t i = 0; i < 2; ++i) {
                world.teams()[i].team_id(teams_[i].team_id);
                world.teams()[i].attacks_positive_x(teams_[i].initial_attacks_positive_x);
            }
            world.ball().valid(update.ball_valid);
            world.ball().centre_s(update.ball_centre);
            world.ball().linear_velocity_s(update.ball_velocity);
            world.ball().angular_velocity_s(update.ball_angular_velocity);
            for (const auto& state : update.robots) {
                nusim_msgs::msg::dds_::RobotTruth_ robot;
                robot.identity().robot_id(state.identity.robot_id);
                robot.identity().model_index(state.identity.model_index);
                robot.identity().team_id(state.identity.team_id);
                robot.identity().player_id(state.identity.player_id);
                robot.torso().position_s({state.base.x, state.base.y, state.base.z});
                robot.torso().orientation_body_to_s(state.base.quat);
                robot.torso().linear_velocity_s(state.base.lin_vel);
                robot.torso().angular_velocity_s(state.base.ang_vel);
                for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                    robot.joints()[i].q(state.joints[i].q);
                    robot.joints()[i].dq(state.joints[i].dq);
                    robot.joints()[i].ddq(state.joints[i].ddq);
                    robot.joints()[i].tau(state.joints[i].tau);
                }
                robot.imu().orientation_imu_to_s(state.imu.quat);
                robot.imu().gyro_imu(state.imu.gyro);
                robot.imu().acceleration_imu(state.imu.acc);
                robot.head().valid(state.head.valid);
                robot.head().position_footprint(state.head.position);
                robot.head().orientation_head_to_footprint(state.head.quat);
                robot.mode(state.mode);
                robot.fall_state(state.fall_state);
                robot.getting_up(state.getting_up);
                world.robots().push_back(robot);
            }
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
                        connection.truth_writer->write(&world);
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
