// WS-Supervisor acceptance: the GameController packet parser and the MuJoCo
// body-placement logic it drives.
//
// Model path resolution mirrors test_model_load.cpp / test_pd_stand.cpp:
// $K1SIM_TEST_MODEL overrides the scene of config/simulation.yaml's default `field`.
//
// Four independent checks:
//   A. Wire-format parsing: a hand-built raw byte buffer (offsets computed
//      independently of the GameControllerPacket struct definition, i.e. not
//      just "cast a live struct back to itself") round-trips through
//      try_parse() with the right header/version/state/kicking_team/team/
//      player fields; malformed headers/versions/lengths are rejected.
//   B. Placement primitives on the real scene: place_free_body_by_geom_center
//      lands the ball's *geom* (not its body origin -- see
//      SupervisorPlacement.hpp for why those differ for this scene's "ball"
//      body) at the field centre at the correct resting height, verified
//      both algebraically and via mj_kinematics; place_free_body lands the
//      robot's root exactly on the requested pose. Both zero residual qvel.
//   C. SupervisorLogic end-to-end, loading the real config/supervisor.yaml:
//      a single State=PLAYING packet (the acceptance scenario) centres the
//      ball; a repeated identical packet is a no-op (edge-triggered, not
//      level-triggered); a per-player PenaltyState transition moves the
//      configured robot body to its penalty_pose / home_pose.
//   D. Kickoff transition truth table across INITIAL->READY->SET->PLAYING:
//      the ball resets on entering READY and on entering PLAYING, not on
//      entering SET.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mujoco/mujoco.h>
#include <string>
#include <vector>

#include "module/Supervisor/src/GameControllerPacket.hpp"
#include "module/Supervisor/src/SupervisorConfig.hpp"
#include "module/Supervisor/src/SupervisorLogic.hpp"
#include "module/Supervisor/src/SupervisorPlacement.hpp"
#include "shared/message/Commands.hpp"
#include "shared/sim/FreeBody.hpp"
#include "shared/util/Config.hpp"

namespace sup = k1sim::module::supervisor;
namespace gc  = k1sim::module::supervisor::gc;

namespace {

    bool g_ok = true;

    void fail(const std::string& what) {
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
        g_ok = false;
    }

    bool approx(double a, double b, double eps = 1e-9) {
        return std::fabs(a - b) <= eps;
    }

    std::string resolve_test_model_path() {
        if (const char* override_path = std::getenv("K1SIM_TEST_MODEL")) {
            return override_path;
        }
        auto cfg = k1sim::config::load("simulation.yaml");
        return k1sim::config::resolve_path(k1sim::config::field_scene(cfg)).string();
    }

    bool actions_mention(const std::vector<sup::SupervisorLogic::Action>& actions, const std::string& needle) {
        for (const auto& a : actions) {
            if (a.message.find(needle) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    // --- A. Wire-format parsing -------------------------------------------------
    //
    // Byte offsets below are computed by hand from GameControllerPacket's field
    // order/sizes under #pragma pack(1) (see GameControllerPacket.hpp), *not*
    // derived from sizeof()/offsetof() on the struct itself -- the point is to
    // catch a struct-definition bug (wrong field order/type/size), so the
    // expected layout has to come from an independent source.
    void test_wire_format() {
        constexpr std::size_t OFF_HEADER       = 0;
        constexpr std::size_t OFF_VERSION      = 4;
        constexpr std::size_t OFF_STATE        = 10;
        constexpr std::size_t OFF_KICKING_TEAM = 13;
        constexpr std::size_t OFF_TEAM0        = 18;
        constexpr std::size_t TEAM_FIXED_SIZE  = 10;  // team_id..message_budget
        constexpr std::size_t ROBOT_SIZE       = 3;
        constexpr std::size_t TEAM_SIZE        = TEAM_FIXED_SIZE + gc::MAX_NUM_PLAYERS * ROBOT_SIZE;  // 70
        constexpr std::size_t PACKET_SIZE      = OFF_TEAM0 + 2 * TEAM_SIZE;                           // 158

        if (sizeof(gc::GameControllerPacket) != PACKET_SIZE) {
            fail("sizeof(GameControllerPacket) = " + std::to_string(sizeof(gc::GameControllerPacket)) + ", expected "
                 + std::to_string(PACKET_SIZE) + " (pack(1) / field layout drifted)");
            return;
        }

        std::vector<std::uint8_t> buf(PACKET_SIZE, 0);
        buf[OFF_HEADER + 0]   = 'R';
        buf[OFF_HEADER + 1]   = 'G';
        buf[OFF_HEADER + 2]   = 'm';
        buf[OFF_HEADER + 3]   = 'e';
        buf[OFF_VERSION]      = 20;
        buf[OFF_STATE]        = static_cast<std::uint8_t>(gc::State::PLAYING);
        buf[OFF_KICKING_TEAM] = 42;
        buf[OFF_TEAM0 + 0]    = 7;  // teams[0].team_id
        buf[OFF_TEAM0 + TEAM_FIXED_SIZE + 0] =
            static_cast<std::uint8_t>(gc::PenaltyState::PICK_UP);  // teams[0].players[0].penalty_state

        gc::GameControllerPacket parsed{};
        if (!gc::try_parse(buf.data(), buf.size(), parsed)) {
            fail("try_parse rejected a well-formed minimal GC packet");
            return;
        }
        if (parsed.state != gc::State::PLAYING) {
            fail("parsed.state != PLAYING");
        }
        if (parsed.kicking_team != 42) {
            fail("parsed.kicking_team != 42");
        }
        if (parsed.teams[0].team_id != 7) {
            fail("parsed.teams[0].team_id != 7");
        }
        if (parsed.teams[0].players[0].penalty_state != gc::PenaltyState::PICK_UP) {
            fail("parsed.teams[0].players[0].penalty_state != PICK_UP");
        }

        // Negative cases: all must be rejected, not crash / not silently parse.
        gc::GameControllerPacket dummy{};
        std::vector<std::uint8_t> bad_header = buf;
        bad_header[0]                        = 'X';
        if (gc::try_parse(bad_header.data(), bad_header.size(), dummy)) {
            fail("try_parse accepted a bad header");
        }
        std::vector<std::uint8_t> bad_version = buf;
        bad_version[OFF_VERSION]              = 1;
        if (gc::try_parse(bad_version.data(), bad_version.size(), dummy)) {
            fail("try_parse accepted an unsupported version");
        }
        if (gc::try_parse(buf.data(), buf.size() - 1, dummy)) {
            fail("try_parse accepted a truncated packet");
        }
        if (gc::try_parse(nullptr, 0, dummy)) {
            fail("try_parse accepted a null buffer");
        }

        std::printf("test_wire_format OK (sizeof(GameControllerPacket)=%zu)\n", sizeof(gc::GameControllerPacket));
    }

    // --- B. Placement primitives ------------------------------------------------
    void test_placement_primitives(const mjModel* m, mjData* d) {
        const int ball_body = mj_name2id(m, mjOBJ_BODY, "ball");
        const int ball_geom = mj_name2id(m, mjOBJ_GEOM, "ball");
        if (ball_body < 0 || ball_geom < 0) {
            fail("scene is missing the 'ball' body/geom");
            return;
        }
        const int ball_jnt      = m->body_jntadr[ball_body];
        const int ball_qpos_adr = m->jnt_qposadr[ball_jnt];
        const int ball_dof_adr  = m->jnt_dofadr[ball_jnt];
        const double radius     = m->geom_size[3 * ball_geom + 0];

        // Dirty the ball's velocity so zeroing is actually exercised, not
        // trivially true because it started at zero.
        for (int i = 0; i < 6; ++i) {
            d->qvel[ball_dof_adr + i] = 3.5;
        }

        if (!sup::place_free_body_by_geom_center(m, d, ball_body, ball_geom, 0.0, 0.0, radius)) {
            fail("place_free_body_by_geom_center returned false for the ball");
            return;
        }

        // Algebraic check: qpos_xyz + the geom's own local offset must equal the
        // requested world point (identity orientation, so no rotation to apply).
        const double gx = m->geom_pos[3 * ball_geom + 0];
        const double gy = m->geom_pos[3 * ball_geom + 1];
        const double gz = m->geom_pos[3 * ball_geom + 2];
        const double wx = d->qpos[ball_qpos_adr + 0] + gx;
        const double wy = d->qpos[ball_qpos_adr + 1] + gy;
        const double wz = d->qpos[ball_qpos_adr + 2] + gz;
        if (!approx(wx, 0.0) || !approx(wy, 0.0) || !approx(wz, radius)) {
            fail("ball geom-center world position != (0,0,radius): got (" + std::to_string(wx) + ", "
                 + std::to_string(wy) + ", " + std::to_string(wz) + "), radius=" + std::to_string(radius));
        }

        // Kinematic check: let MuJoCo itself compute geom_xpos from qpos and
        // compare against the same target -- this is the actually-meaningful
        // physical claim ("the ball ends up at field centre"), not just that our
        // own offset arithmetic is self-consistent.
        mj_kinematics(m, d);
        const double kx = d->geom_xpos[3 * ball_geom + 0];
        const double ky = d->geom_xpos[3 * ball_geom + 1];
        const double kz = d->geom_xpos[3 * ball_geom + 2];
        if (!approx(kx, 0.0, 1e-6) || !approx(ky, 0.0, 1e-6) || !approx(kz, radius, 1e-6)) {
            fail("mj_kinematics ball geom_xpos != (0,0,radius): got (" + std::to_string(kx) + ", " + std::to_string(ky)
                 + ", " + std::to_string(kz) + ")");
        }

        for (int i = 0; i < 6; ++i) {
            if (d->qvel[ball_dof_adr + i] != 0.0) {
                fail("ball qvel not zeroed by placement (dof " + std::to_string(i) + ")");
                break;
            }
        }

        // Robot root: no geom-offset quirk, qpos_xyz is the requested pose
        // directly (see k1_scene_robocup.xml's "kickoff" keyframe, which uses
        // this exact convention).
        const int trunk_body = mj_name2id(m, mjOBJ_BODY, "Trunk");
        if (trunk_body < 0) {
            fail("scene is missing the 'Trunk' body");
            return;
        }
        const int trunk_jnt      = m->body_jntadr[trunk_body];
        const int trunk_qpos_adr = m->jnt_qposadr[trunk_jnt];
        const int trunk_dof_adr  = m->jnt_dofadr[trunk_jnt];
        for (int i = 0; i < 6; ++i) {
            d->qvel[trunk_dof_adr + i] = -2.0;
        }

        const double yaw = 1.5707963267948966;  // pi/2 -- avoid relying on M_PI's availability under -std=c++17
        if (!sup::place_free_body(m, d, trunk_body, -1.0, 2.0, 0.6, yaw)) {
            fail("place_free_body returned false for Trunk");
            return;
        }
        const auto q = sup::yaw_to_quat(yaw);
        bool pose_ok = approx(d->qpos[trunk_qpos_adr + 0], -1.0) && approx(d->qpos[trunk_qpos_adr + 1], 2.0)
                       && approx(d->qpos[trunk_qpos_adr + 2], 0.6) && approx(d->qpos[trunk_qpos_adr + 3], q[0])
                       && approx(d->qpos[trunk_qpos_adr + 4], q[1]) && approx(d->qpos[trunk_qpos_adr + 5], q[2])
                       && approx(d->qpos[trunk_qpos_adr + 6], q[3]);
        if (!pose_ok) {
            fail("Trunk qpos after place_free_body doesn't match the requested pose");
        }
        for (int i = 0; i < 6; ++i) {
            if (d->qvel[trunk_dof_adr + i] != 0.0) {
                fail("Trunk qvel not zeroed by placement (dof " + std::to_string(i) + ")");
                break;
            }
        }

        if (g_ok) {
            std::printf("test_placement_primitives OK (ball radius=%.4f)\n", radius);
        }
    }

    // --- C. SupervisorLogic end-to-end, real config/supervisor.yaml ------------
    void test_supervisor_logic(const mjModel* m, mjData* d) {
        sup::SupervisorConfig cfg = sup::load_config(k1sim::config::load("supervisor.yaml"));
        if (cfg.robots.empty()) {
            fail("config/supervisor.yaml has no robots[] entries -- test needs at least one");
            return;
        }
        // SupervisorLogic's constructor takes its config by value (and moves
        // that parameter copy into itself) -- passing the `cfg` lvalue below
        // copies it, so `cfg` (and this reference into it) stays valid after.
        const sup::RobotConfig& robot_cfg = cfg.robots.front();

        const int ball_body  = mj_name2id(m, mjOBJ_BODY, cfg.ball.body.c_str());
        const int ball_geom  = mj_name2id(m, mjOBJ_GEOM, cfg.ball.geom.c_str());
        const int robot_body = mj_name2id(m, mjOBJ_BODY, robot_cfg.body.c_str());
        if (ball_body < 0 || ball_geom < 0 || robot_body < 0) {
            fail("configured ball/robot body or geom not found in the model");
            return;
        }
        const int ball_jnt       = m->body_jntadr[ball_body];
        const int ball_qpos_adr  = m->jnt_qposadr[ball_jnt];
        const double radius      = m->geom_size[3 * ball_geom + 0];
        const int robot_jnt      = m->body_jntadr[robot_body];
        const int robot_qpos_adr = m->jnt_qposadr[robot_jnt];

        sup::SupervisorLogic logic(cfg);

        // The acceptance scenario: a single State=PLAYING packet (first packet
        // this SupervisorLogic has ever seen -- old state is the UNKNOWN_STATE
        // sentinel) must centre the ball.
        gc::GameControllerPacket pkt{};
        pkt.header       = gc::RECEIVE_HEADER;
        pkt.version      = gc::SUPPORTED_VERSION;
        pkt.state        = gc::State::PLAYING;
        pkt.kicking_team = 0;

        auto actions = logic.process(m, d, pkt);

        const double bx = d->qpos[ball_qpos_adr + 0] + m->geom_pos[3 * ball_geom + 0];
        const double by = d->qpos[ball_qpos_adr + 1] + m->geom_pos[3 * ball_geom + 1];
        const double bz = d->qpos[ball_qpos_adr + 2] + m->geom_pos[3 * ball_geom + 2];
        if (!approx(bx, cfg.ball.x) || !approx(by, cfg.ball.y) || !approx(bz, radius)) {
            fail("SupervisorLogic: State=PLAYING (first packet) did not centre the ball; got (" + std::to_string(bx)
                 + ", " + std::to_string(by) + ", " + std::to_string(bz) + ")");
        }
        if (!actions_mention(actions, "ball")) {
            fail("SupervisorLogic: no ball-placement action reported for the kickoff packet");
        }

        // No-op check: mark the ball with a recognisable position that isn't the
        // centre, then re-run the *same* State=PLAYING packet (no transition --
        // still PLAYING) and confirm the marker survives untouched.
        sup::place_free_body(m, d, ball_body, 5.0, 5.0, 5.0, 0.0);
        logic.process(m, d, pkt);
        if (!approx(d->qpos[ball_qpos_adr + 0], 5.0) || !approx(d->qpos[ball_qpos_adr + 1], 5.0)
            || !approx(d->qpos[ball_qpos_adr + 2], 5.0)) {
            fail("SupervisorLogic: a repeated (non-transitioning) PLAYING packet moved the ball (should be a no-op)");
        }

        // Penalty transition: this player becomes penalised -> robot goes to
        // penalty_pose; then unpenalised -> back to home_pose. Uses whichever
        // packet slot/player index the config actually points at.
        gc::Team* team_ptr = nullptr;
        if (robot_cfg.team_id >= 0) {
            for (auto& t : pkt.teams) {
                if (t.team_id == static_cast<std::uint8_t>(robot_cfg.team_id)) {
                    team_ptr = &t;
                    break;
                }
            }
        }
        if (team_ptr == nullptr) {
            team_ptr = &pkt.teams[static_cast<std::size_t>(robot_cfg.team_index)];
        }
        gc::Team& team                                                                = *team_ptr;
        team.players[static_cast<std::size_t>(robot_cfg.player_id) - 1].penalty_state = gc::PenaltyState::PICK_UP;

        logic.process(m, d, pkt);
        bool at_penalty = approx(d->qpos[robot_qpos_adr + 0], robot_cfg.penalty_pose.x)
                          && approx(d->qpos[robot_qpos_adr + 1], robot_cfg.penalty_pose.y)
                          && approx(d->qpos[robot_qpos_adr + 2], robot_cfg.penalty_pose.z);
        if (!at_penalty) {
            fail("SupervisorLogic: penalised robot was not moved to its configured penalty_pose");
        }

        team.players[static_cast<std::size_t>(robot_cfg.player_id) - 1].penalty_state = gc::PenaltyState::UNPENALISED;
        logic.process(m, d, pkt);
        bool at_home = approx(d->qpos[robot_qpos_adr + 0], robot_cfg.home_pose.x)
                       && approx(d->qpos[robot_qpos_adr + 1], robot_cfg.home_pose.y)
                       && approx(d->qpos[robot_qpos_adr + 2], robot_cfg.home_pose.z);
        if (!at_home) {
            fail("SupervisorLogic: unpenalised robot was not moved back to its configured home_pose");
        }

        if (g_ok) {
            std::printf("test_supervisor_logic OK (robot body '%s', %zu action(s) on kickoff)\n",
                        robot_cfg.body.c_str(),
                        actions.size());
        }
    }

    // --- D. Kickoff transition truth table --------------------------------------
    void test_kickoff_transitions(const mjModel* m, mjData* d) {
        sup::SupervisorConfig cfg = sup::load_config(k1sim::config::load("supervisor.yaml"));
        const int ball_body       = mj_name2id(m, mjOBJ_BODY, cfg.ball.body.c_str());
        const int ball_geom       = mj_name2id(m, mjOBJ_GEOM, cfg.ball.geom.c_str());
        const int ball_jnt        = m->body_jntadr[ball_body];
        const int ball_qpos_adr   = m->jnt_qposadr[ball_jnt];
        const double radius       = m->geom_size[3 * ball_geom + 0];

        sup::SupervisorLogic logic(cfg);

        auto mark      = [&] { sup::place_free_body(m, d, ball_body, -9.0, -9.0, -9.0, 0.0); };
        auto at_centre = [&] {
            const double x = d->qpos[ball_qpos_adr + 0] + m->geom_pos[3 * ball_geom + 0];
            const double y = d->qpos[ball_qpos_adr + 1] + m->geom_pos[3 * ball_geom + 1];
            const double z = d->qpos[ball_qpos_adr + 2] + m->geom_pos[3 * ball_geom + 2];
            return approx(x, cfg.ball.x) && approx(y, cfg.ball.y) && approx(z, radius);
        };

        auto step = [&](gc::State state, bool expect_reset, const char* label) {
            mark();
            gc::GameControllerPacket pkt{};
            pkt.header  = gc::RECEIVE_HEADER;
            pkt.version = gc::SUPPORTED_VERSION;
            pkt.state   = state;
            logic.process(m, d, pkt);
            const bool reset = at_centre();
            if (reset != expect_reset) {
                fail(std::string("kickoff transition '") + label + "': expected reset="
                     + (expect_reset ? "true" : "false") + ", got " + (reset ? "true" : "false"));
            }
        };

        // INITIAL (bootstrap, no reset expected -- entering INITIAL isn't a
        // kickoff trigger) -> READY (reset) -> SET (no reset) -> PLAYING (reset).
        step(gc::State::INITIAL, false, "start->INITIAL");
        step(gc::State::READY, true, "INITIAL->READY");
        step(gc::State::SET, false, "READY->SET");
        step(gc::State::PLAYING, true, "SET->PLAYING");
        step(gc::State::PLAYING, false, "PLAYING->PLAYING (steady state)");

        if (g_ok) {
            std::printf("test_kickoff_transitions OK\n");
        }
    }


    // rt/nusim/ball_command: placement, centre velocity through the off-origin geom, rolling spin,
    // the robot-relative frame, and that a rolled ball keeps its speed instead of skidding.
    void test_ball_command(const mjModel* m, mjData* d) {
        namespace fb        = k1sim::freebody;
        using Frame         = k1sim::message::BallCommand::Frame;
        const int ball_body = mj_name2id(m, mjOBJ_BODY, "ball");
        const int ball_geom = mj_name2id(m, mjOBJ_GEOM, "ball");
        const int trunk     = mj_name2id(m, mjOBJ_BODY, "Trunk");
        if (ball_body < 0 || ball_geom < 0 || trunk < 0) {
            fail("scene is missing the ball body/geom or the Trunk");
            return;
        }
        const double radius = m->geom_size[3 * ball_geom + 0];
        mj_resetData(m, d);

        // World frame, resting on the floor, rolling
        k1sim::message::BallCommand cmd;
        cmd.frame    = Frame::WORLD;
        cmd.position = {2.0, 0.5, -1.0};
        cmd.velocity = {-3.0, 0.5, 0.0};
        cmd.rolling  = true;
        if (!sup::apply_ball_command(m, d, ball_body, ball_geom, trunk, cmd)) {
            fail("apply_ball_command returned false for a valid world command");
            return;
        }
        const auto state = fb::geom_centre_state(m, d, ball_body, ball_geom);
        if (!approx(state.position[0], 2.0) || !approx(state.position[1], 0.5) || !approx(state.position[2], radius)) {
            fail("ball command: centre not at (2, 0.5, radius)");
        }
        if (!approx(state.lin_vel[0], -3.0) || !approx(state.lin_vel[1], 0.5) || !approx(state.lin_vel[2], 0.0)) {
            fail("ball command: centre velocity != commanded");
        }
        // Rolling without slipping: the contact point r below the centre is at rest
        const double cx = state.lin_vel[0] + state.ang_vel[1] * -radius;
        const double cy = state.lin_vel[1] - state.ang_vel[0] * -radius;
        if (!approx(cx, 0.0, 1e-9) || !approx(cy, 0.0, 1e-9)) {
            fail("ball command: contact point moves, so the ball is not rolling");
        }

        // Cross-check the free-joint arithmetic against MuJoCo's own object velocity at the geom
        mj_forward(m, d);
        mjtNum vel6[6];
        mj_objectVelocity(m, d, mjOBJ_GEOM, ball_geom, vel6, 0);
        if (!approx(vel6[3], -3.0, 1e-6) || !approx(vel6[4], 0.5, 1e-6) || !approx(vel6[5], 0.0, 1e-6)) {
            fail("ball command: mj_objectVelocity at the geom disagrees with the commanded centre velocity (got "
                 + std::to_string(vel6[3]) + ", " + std::to_string(vel6[4]) + ", " + std::to_string(vel6[5]) + ")");
        }

        // A rolled ball keeps (nearly) its speed over 0.2 s instead of skidding and losing a chunk of it
        const double speed0 = std::hypot(-3.0, 0.5);
        const int steps     = static_cast<int>(0.2 / m->opt.timestep);
        for (int i = 0; i < steps; ++i) {
            mj_step(m, d);
        }
        const auto rolled = fb::geom_centre_state(m, d, ball_body, ball_geom);
        const double speed = std::hypot(rolled.lin_vel[0], rolled.lin_vel[1]);
        if (speed < 0.9 * speed0 || speed > speed0 * 1.001) {
            fail("rolled ball speed after 0.2 s is " + std::to_string(speed) + " (started at " + std::to_string(speed0)
                 + "): expected only rolling-resistance loss");
        }
        if (rolled.position[0] > 2.0 - 0.5) {
            fail("rolled ball did not travel in the commanded direction");
        }

        // Robot frame: Trunk at (1, 2) yawed +90 deg, ball 1 m in front moving forward
        mj_resetData(m, d);
        const int tq = m->jnt_qposadr[m->body_jntadr[trunk]];
        d->qpos[tq + 0] = 1.0;
        d->qpos[tq + 1] = 2.0;
        d->qpos[tq + 3] = std::cos(M_PI / 4.0);
        d->qpos[tq + 4] = 0.0;
        d->qpos[tq + 5] = 0.0;
        d->qpos[tq + 6] = std::sin(M_PI / 4.0);
        cmd.frame       = Frame::ROBOT;
        cmd.position    = {1.0, 0.0, -1.0};
        cmd.velocity    = {1.0, 0.0, 0.0};
        cmd.rolling     = false;
        cmd.angular_velocity = {0.0, 0.0, 0.0};
        if (!sup::apply_ball_command(m, d, ball_body, ball_geom, trunk, cmd)) {
            fail("apply_ball_command returned false for a valid robot command");
            return;
        }
        const auto rel = fb::geom_centre_state(m, d, ball_body, ball_geom);
        if (!approx(rel.position[0], 1.0, 1e-9) || !approx(rel.position[1], 3.0, 1e-9)) {
            fail("robot-frame ball command: centre not at world (1, 3)");
        }
        if (!approx(rel.lin_vel[0], 0.0, 1e-9) || !approx(rel.lin_vel[1], 1.0, 1e-9)) {
            fail("robot-frame ball command: velocity not rotated into world (0, 1)");
        }

        // Invalid ball, and a robot-relative command without a free robot body
        if (sup::apply_ball_command(m, d, -1, ball_geom, trunk, cmd)) {
            fail("apply_ball_command accepted an invalid ball body");
        }
        if (sup::apply_ball_command(m, d, ball_body, ball_geom, -1, cmd)) {
            fail("apply_ball_command accepted a robot-relative command without a robot body");
        }
        mj_resetData(m, d);
    }


    // Re-originating the ball must not move it in any keyframe: every inherited keyframe zero-pads
    // the ball, which used to mean "body at the origin, sphere at its offset spot".
    void test_ball_keyframes(const mjModel* m, mjData* d) {
        const int ball_geom = mj_name2id(m, mjOBJ_GEOM, "ball");
        const int ball_body = mj_name2id(m, mjOBJ_BODY, "ball");
        if (ball_geom < 0 || ball_body < 0) {
            fail("scene is missing the ball");
            return;
        }
        for (double v : {m->geom_pos[3 * ball_geom], m->geom_pos[3 * ball_geom + 1], m->geom_pos[3 * ball_geom + 2]}) {
            if (!approx(v, 0.0)) {
                fail("ball geom still offset from its body origin after re-originating");
            }
        }
        for (int key = 0; key < m->nkey; ++key) {
            mj_resetDataKeyframe(m, d, key);
            mj_kinematics(m, d);
            const double* p = d->geom_xpos + 3 * ball_geom;
            if (!approx(p[0], 1.38, 1e-9) || !approx(p[1], 0.0, 1e-9) || !approx(p[2], 0.0785, 1e-9)) {
                fail(std::string("keyframe '") + mj_id2name(m, mjOBJ_KEY, key) + "' no longer rests the ball at (1.38, 0, 0.0785)");
            }
        }
        mj_resetData(m, d);
    }

}  // namespace

int main() {
    test_wire_format();

    const std::string model_path = resolve_test_model_path();
    char error[1024]             = {0};
    // Load the scene the way SimCore does: through the spec, with the ball body re-originated on its
    // sphere (freebody::reorigin_body_at_geom) and the keyframes shifted to match.
    mjSpec* spec = mj_parseXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (spec == nullptr) {
        std::fprintf(stderr, "mj_parseXML failed for '%s': %s\n", model_path.c_str(), error);
        return 1;
    }
    const auto ball_offset = k1sim::freebody::reorigin_body_at_geom(spec, "ball", "ball");
    mjModel* m             = mj_compile(spec, nullptr);
    mj_deleteSpec(spec);
    if (m == nullptr) {
        std::fprintf(stderr, "mj_compile failed for '%s'\n", model_path.c_str());
        return 1;
    }
    k1sim::freebody::shift_reoriginated_keyframes(m, mj_name2id(m, mjOBJ_BODY, "ball"), ball_offset);
    if (ball_offset[0] == 0.0 && ball_offset[1] == 0.0 && ball_offset[2] == 0.0) {
        fail("expected the scene ball to be re-originated (its geom sits off the body origin)");
    }
    mjData* d = mj_makeData(m);
    if (d == nullptr) {
        std::fprintf(stderr, "mj_makeData failed\n");
        mj_deleteModel(m);
        return 1;
    }

    test_placement_primitives(m, d);
    test_supervisor_logic(m, d);
    test_kickoff_transitions(m, d);
    test_ball_command(m, d);
    test_ball_keyframes(m, d);

    mj_deleteData(d);
    mj_deleteModel(m);

    if (g_ok) {
        std::printf("test_supervisor OK\n");
    }
    return g_ok ? 0 : 1;
}
