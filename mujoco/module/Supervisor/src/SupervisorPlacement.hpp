#ifndef K1SIM_MODULE_SUPERVISOR_SUPERVISORPLACEMENT_HPP
#define K1SIM_MODULE_SUPERVISOR_SUPERVISORPLACEMENT_HPP

#include <array>
#include <cmath>
#include <mujoco/mujoco.h>

#include "shared/message/Commands.hpp"
#include "shared/sim/FreeBody.hpp"

// Pure MuJoCo qpos/qvel writers for teleporting free-jointed bodies (the ball,
// or a robot's root) — the physics-placement primitive the sim-side
// GameController supervisor uses to mirror Webots' Supervisor role (ball to
// centre on kickoff, penalised robots off to the side line, etc).
//
// Nothing here touches the sim mutex (message::SimHandles::mutex) — callers
// (Supervisor.cpp) must hold it for the duration of the write, exactly like
// Viewer.cpp's perturb-force application does. That's also what makes these
// functions directly unit-testable: a test can load a model, build an mjData
// with mj_makeData and call these with no NUClear/threading involved at all.
namespace k1sim::module::supervisor {

    // yaw-only orientation (about world +Z) as a MuJoCo wxyz quaternion — all the
    // placement poses this module deals with (kickoff spawn, penalty spot) are
    // flat-ground standing poses, so pitch/roll are always zero.
    inline std::array<double, 4> yaw_to_quat(double yaw_rad) {
        const double half = 0.5 * yaw_rad;
        return {std::cos(half), 0.0, 0.0, std::sin(half)};
    }

    // Writes body_id's free-joint qpos to the given world position + yaw and
    // zeros its qvel (both linear and angular), so the body is dropped in at
    // rest with no inherited velocity. body_id must own exactly one joint, and it
    // must be a free joint (mjJNT_FREE) — true for the ball body and the robot's
    // "Trunk" body in k1_scene_robocup.xml, per that file and K1_22dof.xml.
    // Returns false (no write) if body_id is invalid or isn't free-jointed, so
    // callers can log a config error instead of corrupting unrelated qpos.
    inline bool
        place_free_body(const mjModel* m, mjData* d, int body_id, double x, double y, double z, double yaw_rad) {
        if (m == nullptr || d == nullptr || body_id < 0 || body_id >= m->nbody) {
            return false;
        }
        if (m->body_jntnum[body_id] != 1) {
            return false;
        }
        const int jnt = m->body_jntadr[body_id];
        if (m->jnt_type[jnt] != mjJNT_FREE) {
            return false;
        }

        const int qpos_adr = m->jnt_qposadr[jnt];
        const int dof_adr  = m->jnt_dofadr[jnt];
        const auto q       = yaw_to_quat(yaw_rad);

        d->qpos[qpos_adr + 0] = x;
        d->qpos[qpos_adr + 1] = y;
        d->qpos[qpos_adr + 2] = z;
        d->qpos[qpos_adr + 3] = q[0];
        d->qpos[qpos_adr + 4] = q[1];
        d->qpos[qpos_adr + 5] = q[2];
        d->qpos[qpos_adr + 6] = q[3];
        for (int i = 0; i < 6; ++i) {
            d->qvel[dof_adr + i] = 0.0;
        }
        return true;
    }

    // Like place_free_body, but the target (x,y,z) is the world-space centre of
    // one of the body's own geoms rather than the body origin itself — needed for
    // k1_scene_robocup.xml's "ball" body, whose sphere geom is deliberately kept
    // off the body origin (a freejoint-keyframe zero-padding workaround; see that
    // file's header comment). Solves target = qpos_xyz + geom_local_pos for
    // qpos_xyz (valid because the ball geom has no <geom quat=...>, i.e. zero
    // rotation relative to its body, and this function always places the body at
    // identity orientation — a rotated offset would need the body's orientation
    // folded in too, which no current use case needs). For a geom that already
    // sits at its body's origin this is exactly place_free_body(..., yaw=0).
    inline bool place_free_body_by_geom_center(const mjModel* m,
                                               mjData* d,
                                               int body_id,
                                               int geom_id,
                                               double x,
                                               double y,
                                               double z) {
        if (m == nullptr || geom_id < 0 || geom_id >= m->ngeom || m->geom_bodyid[geom_id] != body_id) {
            return false;
        }
        const double ox = m->geom_pos[3 * geom_id + 0];
        const double oy = m->geom_pos[3 * geom_id + 1];
        const double oz = m->geom_pos[3 * geom_id + 2];
        return place_free_body(m, d, body_id, x - ox, y - oy, z - oz, 0.0);
    }


    // Applies a NUSim ball command (rt/nusim/ball_command, see shared/k1/NUSimApi.hpp): places the
    // ball geom's centre and sets its velocity and spin. For Frame::ROBOT the position, velocity and
    // spin are expressed in the yaw-only frame at robot_body's ground projection (x forward, z up),
    // so a test can roll the ball at the robot wherever it stands. position.z < 0 rests the ball on
    // the floor at its radius. rolling derives the spin for rolling without slipping from the
    // (world) velocity and ignores angular_velocity, so the ball does not skid and lose speed on
    // its first contact. Returns false (no write) for an invalid ball, or a missing/non-free robot
    // body when the command is robot-relative. Caller holds the sim mutex.
    inline bool apply_ball_command(const mjModel* m,
                                   mjData* d,
                                   int ball_body_id,
                                   int ball_geom_id,
                                   int robot_body_id,
                                   const k1sim::message::BallCommand& cmd) {
        if (!k1sim::freebody::valid_free_body_geom(m, ball_body_id, ball_geom_id)) {
            return false;
        }

        // Robot-relative commands: rotate by the robot's yaw and offset by its ground projection
        double ox = 0.0, oy = 0.0, c = 1.0, s = 0.0;
        if (cmd.frame == k1sim::message::BallCommand::Frame::ROBOT) {
            if (robot_body_id < 0 || robot_body_id >= m->nbody || m->body_jntnum[robot_body_id] != 1
                || m->jnt_type[m->body_jntadr[robot_body_id]] != mjJNT_FREE) {
                return false;
            }
            const mjtNum* q = d->qpos + m->jnt_qposadr[m->body_jntadr[robot_body_id]];
            ox              = q[0];
            oy              = q[1];
            // yaw of the wxyz quaternion q[3..6]
            const double yaw = std::atan2(2.0 * (q[3] * q[6] + q[4] * q[5]), 1.0 - 2.0 * (q[5] * q[5] + q[6] * q[6]));
            c                = std::cos(yaw);
            s                = std::sin(yaw);
        }
        const auto rotate = [c, s](const std::array<double, 3>& v) -> std::array<double, 3> {
            return {c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]};
        };

        const double radius = m->geom_size[3 * ball_geom_id];
        const auto p        = rotate(cmd.position);
        const double x      = ox + p[0];
        const double y      = oy + p[1];
        const double z      = cmd.position[2] < 0.0 ? radius : cmd.position[2];
        if (!place_free_body_by_geom_center(m, d, ball_body_id, ball_geom_id, x, y, z)) {
            return false;
        }

        const auto v = rotate(cmd.velocity);
        const auto w = cmd.rolling ? k1sim::freebody::rolling_spin(v, radius) : rotate(cmd.angular_velocity);
        return k1sim::freebody::set_geom_centre_velocity(m, d, ball_body_id, ball_geom_id, v, w);
    }

}  // namespace k1sim::module::supervisor

#endif  // K1SIM_MODULE_SUPERVISOR_SUPERVISORPLACEMENT_HPP
