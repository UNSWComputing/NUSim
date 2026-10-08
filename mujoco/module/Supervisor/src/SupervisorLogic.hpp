#ifndef K1SIM_MODULE_SUPERVISOR_SUPERVISORLOGIC_HPP
#define K1SIM_MODULE_SUPERVISOR_SUPERVISORLOGIC_HPP

#include <cstdio>
#include <mujoco/mujoco.h>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "module/Supervisor/src/GameControllerPacket.hpp"
#include "module/Supervisor/src/SupervisorConfig.hpp"
#include "module/Supervisor/src/SupervisorPlacement.hpp"

// The sim-side GameController supervisor's decision logic: given the previous
// and current GC packet, decides which bodies need to move and does it.
// Deliberately free of NUClear (no Reactor, no logging, no UDP) so it can be
// driven directly from test_supervisor.cpp against a loaded mjModel/mjData —
// Supervisor.cpp (the Reactor) just owns one of these, feeds it packets
// parsed off the wire, holds the sim mutex around process(), and forwards the
// returned Actions to its own log<>() calls.
namespace k1sim::module::supervisor {

    class SupervisorLogic {
    public:
        explicit SupervisorLogic(SupervisorConfig cfg) : cfg_(std::move(cfg)) {}

        struct Action {
            enum class Level { INFO, WARN } level = Level::INFO;
            std::string message;
            int robot_id = 0;
        };

        // Diffs `pkt` against the last packet seen (or, on the very first call,
        // against the same "nothing has happened yet" baseline NUbots'
        // GameController::reset_state() bootstraps: an unknown/sentinel State so
        // any real State counts as a transition, and UNPENALISED for every
        // player), applies whatever body placements that diff implies directly to
        // `d`, and returns a log of what happened (empty if nothing did).
        //
        // Caller must hold the sim mutex (message::SimHandles::mutex) for the
        // duration of this call — it writes mjData qpos/qvel with no locking of
        // its own, same contract as the SupervisorPlacement.hpp functions it
        // calls.
        std::vector<Action> process(const mjModel* m, mjData* d, const gc::GameControllerPacket& pkt) {
            std::vector<Action> actions;

            const gc::State old_state = prev_ ? prev_->state : gc::UNKNOWN_STATE;
            const gc::State new_state = pkt.state;
            const bool state_changed  = new_state != old_state;

            if (state_changed) {
                actions.push_back({Action::Level::INFO, std::string("state -> ") + gc::state_name(new_state)});
            }

            if (!prev_ || prev_->kicking_team != pkt.kicking_team) {
                // Logged only: which team has kickoff doesn't change where the
                // ball goes (always centre), see the task brief this module was
                // built against. Kept here because a future robot-positioning
                // rule (kicking team must stay outside the centre circle, etc.)
                // would need it, and it's what "parse enough to know kickoff
                // team" (this module's spec) means in the meantime.
                actions.push_back({Action::Level::INFO, "kickoff team id -> " + std::to_string(pkt.kicking_team)});
            }

            // Core kickoff rule: ball -> centre the instant we enter READY
            // (teams are now walking to kickoff positions with a known ball spot
            // — this is also what real matches/Webots show) or PLAYING (SET->
            // PLAYING is the literal kickoff whistle). Using "entered PLAYING"
            // rather than requiring specifically SET->PLAYING also covers the
            // bootstrap case where the first packet this process ever observes
            // already reports PLAYING (old_state is the UNKNOWN_STATE sentinel,
            // which never equals PLAYING, so this still fires).
            bool center_ball = state_changed && (new_state == gc::State::READY || new_state == gc::State::PLAYING);

            // Optional extra resets (task brief: "On FINISHED/goal/half
            // (optional): reset ball to centre").
            if (cfg_.reset_ball_on_finished && state_changed && new_state == gc::State::FINISHED) {
                center_ball = true;
            }
            if (prev_) {
                if (cfg_.reset_ball_on_goal && score_increased(*prev_, pkt)) {
                    center_ball = true;
                    actions.push_back({Action::Level::INFO, "goal detected"});
                }
                if (cfg_.reset_ball_on_half_change && prev_->first_half != pkt.first_half) {
                    center_ball = true;
                    actions.push_back({Action::Level::INFO, "half changed"});
                }
            }

            if (center_ball && cfg_.ball.enabled) {
                place_ball(m, d, actions);
            }

            for (const auto& rc : cfg_.robots) {
                apply_robot(m, d, rc, pkt, actions);
            }

            prev_ = pkt;
            return actions;
        }

        // Whether process() has seen a packet yet (idle vs live) — purely
        // informational, e.g. for a Startup log line.
        [[nodiscard]] bool has_seen_packet() const {
            return prev_.has_value();
        }

    private:
        static bool score_increased(const gc::GameControllerPacket& old_pkt, const gc::GameControllerPacket& new_pkt) {
            for (std::size_t i = 0; i < old_pkt.teams.size(); ++i) {
                if (new_pkt.teams[i].score > old_pkt.teams[i].score) {
                    return true;
                }
            }
            return false;
        }

        static const gc::Team* resolve_team(const gc::GameControllerPacket& pkt, const RobotConfig& rc) {
            if (rc.team_id >= 0) {
                for (const auto& t : pkt.teams) {
                    if (t.team_id == static_cast<std::uint8_t>(rc.team_id)) {
                        return &t;
                    }
                }
                return nullptr;
            }
            if (rc.team_index == 0 || rc.team_index == 1) {
                return &pkt.teams[static_cast<std::size_t>(rc.team_index)];
            }
            return nullptr;
        }

        void warn_once(std::vector<Action>& actions, const std::string& msg) {
            if (warned_.insert(msg).second) {
                actions.push_back({Action::Level::WARN, msg});
            }
        }

        void place_ball(const mjModel* m, mjData* d, std::vector<Action>& actions) {
            const int body_id = mj_name2id(m, mjOBJ_BODY, cfg_.ball.body.c_str());
            const int geom_id = mj_name2id(m, mjOBJ_GEOM, cfg_.ball.geom.c_str());
            if (body_id < 0 || geom_id < 0) {
                warn_once(actions, "ball body '" + cfg_.ball.body + "' or geom '" + cfg_.ball.geom + "' not in model");
                return;
            }

            double z = cfg_.ball.z;
            if (z < 0.0) {
                z = m->geom_size[3 * geom_id + 0];  // sphere radius -> rests exactly on the floor plane (z=0)
            }

            if (place_free_body_by_geom_center(m, d, body_id, geom_id, cfg_.ball.x, cfg_.ball.y, z)) {
                char buf[128];
                std::snprintf(buf, sizeof(buf), "ball -> centre (%.3f, %.3f, %.3f)", cfg_.ball.x, cfg_.ball.y, z);
                actions.push_back({Action::Level::INFO, buf});
            }
            else {
                warn_once(actions, "ball placement failed (body '" + cfg_.ball.body + "' not free-jointed?)");
            }
        }

        void apply_robot(const mjModel* m,
                         mjData* d,
                         const RobotConfig& rc,
                         const gc::GameControllerPacket& pkt,
                         std::vector<Action>& actions) {
            const gc::Team* new_team = resolve_team(pkt, rc);
            if (new_team == nullptr || rc.player_id < 1
                || static_cast<std::size_t>(rc.player_id) > gc::MAX_NUM_PLAYERS) {
                return;
            }
            const gc::PenaltyState new_ps = new_team->players[static_cast<std::size_t>(rc.player_id) - 1].penalty_state;

            gc::PenaltyState old_ps = gc::PenaltyState::UNPENALISED;
            if (prev_) {
                const gc::Team* old_team = resolve_team(*prev_, rc);
                if (old_team != nullptr) {
                    old_ps = old_team->players[static_cast<std::size_t>(rc.player_id) - 1].penalty_state;
                }
            }

            if (new_ps == old_ps) {
                return;
            }

            const int body_id = mj_name2id(m, mjOBJ_BODY, rc.body.c_str());
            if (body_id < 0) {
                warn_once(actions, "robot body '" + rc.body + "' not in model");
                return;
            }

            if (new_ps != gc::PenaltyState::UNPENALISED) {
                place_free_body(m,
                                d,
                                body_id,
                                rc.penalty_pose.x,
                                rc.penalty_pose.y,
                                rc.penalty_pose.z,
                                rc.penalty_pose.yaw);
                actions.push_back({Action::Level::INFO,
                                   rc.body + " penalised (" + gc::penalty_state_name(new_ps) + ") -> side line",
                                   rc.robot_id});
            }
            else {
                place_free_body(m, d, body_id, rc.home_pose.x, rc.home_pose.y, rc.home_pose.z, rc.home_pose.yaw);
                actions.push_back({Action::Level::INFO, rc.body + " unpenalised -> own half", rc.robot_id});
            }
        }

        SupervisorConfig cfg_;
        std::optional<gc::GameControllerPacket> prev_;
        std::set<std::string> warned_;
    };

}  // namespace k1sim::module::supervisor

#endif  // K1SIM_MODULE_SUPERVISOR_SUPERVISORLOGIC_HPP
