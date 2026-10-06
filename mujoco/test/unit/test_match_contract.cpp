// Validate the roster against real --game ordering, including ambiguous player IDs
// and rejected transport collisions. Exercise the generated snapshot's CDR layout.
#include <algorithm>
#include <cstdio>
#include <exception>
#include <fastrtps/rtps/common/SerializedPayload.h>
#include <string>

#include "WorldSnapshot.h"
#include "WorldSnapshotPubSubTypes.h"
#include "module/Simulation/src/SimCore.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/sim/GroundTruthContract.hpp"
#include "shared/sim/MatchConfig.hpp"
#include "shared/util/Config.hpp"

namespace {

    int failures = 0;

    void expect(bool condition, const std::string& label) {
        if (!condition) {
            std::fprintf(stderr, "FAIL: %s\n", label.c_str());
            ++failures;
        }
    }

    template <typename F>
    void rejects(F operation, const std::string& label) {
        try {
            operation();
            expect(false, label);
        }
        catch (const std::exception&) {
        }
    }

    void check_wire(const k1sim::MatchConfig& match) {
        using namespace nusim_msgs::msg::dds_;
        WorldSnapshot_ sent;
        sent.stamp().schema_version(k1sim::ground_truth::SCHEMA_VERSION);
        const std::string session_id = "98d980b9-1390-4e72-ad24-2eebd777893b";
        std::copy(session_id.begin(), session_id.end(), sent.stamp().session_id().begin());
        sent.stamp().reset_generation(0x100000002ULL);
        sent.stamp().sample_sequence(0x200000003ULL);
        sent.stamp().step_count(120);
        sent.stamp().sim_time(0.12);
        sent.stamp().capture_time_unix_ns(1791244800123456789ULL);
        sent.ball().valid(true);
        sent.ball().centre_s({1.5, -2.5, 0.1});
        sent.ball().linear_velocity_s({-0.5, 0.75, 0.0});
        sent.ball().angular_velocity_s({0.0, 1.0, -1.0});
        for (std::size_t i = 0; i < match.teams.size(); ++i) {
            sent.teams()[i].team_id(match.teams[i].team_id);
            sent.teams()[i].attacks_positive_x(match.teams[i].initial_attacks_positive_x);
        }
        for (const auto& identity : match.robots) {
            RobotTruth_ robot;
            robot.identity().robot_id(identity.robot_id);
            robot.identity().model_index(identity.model_index);
            robot.identity().team_id(identity.team_id);
            robot.identity().player_id(identity.player_id);
            robot.torso().position_s({-3.0 + identity.model_index, 2.0, 0.555});
            robot.torso().orientation_body_to_s({1.0, 0.0, 0.0, 0.0});
            robot.torso().linear_velocity_s({0.1, -0.2, 0.3});
            robot.torso().angular_velocity_s({-0.4, 0.5, -0.6});
            expect(robot.joints().size() == k1sim::JOINT_COUNT, "wire joint array follows JointIndexK1");
            for (std::size_t j = 0; j < k1sim::JOINT_COUNT; ++j) {
                robot.joints()[j].q(0.01 * (j + identity.robot_id));
                robot.joints()[j].dq(-0.02 * j);
                robot.joints()[j].ddq(0.03 * j);
                robot.joints()[j].tau(-0.04 * j);
            }
            robot.imu().orientation_imu_to_s({1.0, 0.0, 0.0, 0.0});
            robot.imu().gyro_imu({0.1, 0.2, 0.3});
            robot.imu().acceleration_imu({0.0, 0.0, 9.81});
            robot.head().valid(true);
            robot.head().position_footprint({0.05, 0.0, 0.8});
            robot.head().orientation_head_to_footprint({1.0, 0.0, 0.0, 0.0});
            robot.mode(3);
            robot.fall_state(0);
            robot.getting_up(false);
            sent.robots().push_back(robot);
        }
        WorldSnapshot_PubSubType type;
        expect(std::string(type.getName()) == k1sim::ground_truth::TYPE_WORLD_SNAPSHOT, "registered DDS type name");
        eprosima::fastrtps::rtps::SerializedPayload_t payload(type.getSerializedSizeProvider(&sent)());
        expect(type.serialize(&sent, &payload), "serialize complete snapshot");
        WorldSnapshot_ received;
        expect(type.deserialize(&payload, &received), "deserialize complete snapshot");
        expect(received == sent, "all six robot states and 64-bit timestamps survive CDR round trip");
    }

}  // namespace

int main() {
    try {
        const auto root  = k1sim::config::load("match_3v3.yaml");
        const auto match = k1sim::load_match_config(root);
        expect(match.robots.size() == 6, "3v3 has six identities");
        expect(match.robots[0].body() == "Trunk", "legacy main robot name");
        expect(match.robots[5].body() == "sub05_Trunk", "sixth scene slot");
        expect(match.robots[0].player_id == match.robots[3].player_id
                   && match.robots[0].robot_id != match.robots[3].robot_id,
               "opposing player 1s have different simulator identities");
        const auto game = k1sim::config::game(k1sim::config::load("simulation.yaml"), 3, true);
        expect(game.spawns.size() == match.robots.size(), "roster agrees with --game 3 model order");
        expect(game.spawns[0][0] < 0.0 && game.spawns[3][0] > 0.0,
               "roster initial team directions match game placement");
        auto changed                 = match;
        changed.robots[3].dds_domain = changed.robots[0].dds_domain;
        rejects([&] { changed.validate(); }, "duplicate DDS domain rejected");
        changed                     = match;
        changed.robots[1].player_id = changed.robots[0].player_id;
        rejects([&] { changed.validate(); }, "duplicate same-team player rejected");
        changed                    = match;
        changed.robots[1].robot_id = 1;
        rejects([&] { changed.validate(); }, "duplicate robot ID rejected");
        changed                       = match;
        changed.robots[1].model_index = 4;
        rejects([&] { changed.validate(); }, "incorrect model order rejected");
        changed                   = match;
        changed.robots[0].team_id = match.teams[1].team_id;
        rejects([&] { changed.validate(); }, "wrong team block rejected");
        changed                      = match;
        changed.robots[0].dds_domain = -1;
        rejects([&] { changed.validate(); }, "negative domain rejected");
        changed.robots[0].dds_domain = 64;
        rejects([&] { changed.validate(); }, "out-of-range domain rejected");
        changed                     = match;
        changed.robots[0].player_id = 12;
        rejects([&] { changed.validate(); }, "out-of-range player rejected");
        changed                                     = match;
        changed.teams[1].initial_attacks_positive_x = changed.teams[0].initial_attacks_positive_x;
        rejects([&] { changed.validate(); }, "matching attack directions rejected");
        changed = match;
        changed.robots.pop_back();
        rejects([&] { changed.validate(); }, "unequal team sizes rejected");
        changed                = match;
        changed.schema_version = 2;
        rejects([&] { changed.validate(); }, "unknown schema rejected");
        auto missing = YAML::Clone(root);
        missing["robots"][0].remove("dds_domain");
        rejects([&] { k1sim::load_match_config(missing); }, "missing domain is not silently defaulted");
        rejects([] { k1sim::robot_prefix(-1); }, "negative model slot rejected");
        expect(k1sim::robot_prefix(21) == "sub21_", "11v11 last model slot supported");
        rejects([] { k1sim::robot_prefix(22); }, "excess model slot rejected");
        check_wire(match);
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
    return failures == 0 ? 0 : 1;
}
