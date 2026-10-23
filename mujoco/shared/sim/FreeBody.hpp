#ifndef K1SIM_SHARED_SIM_FREEBODY_HPP
#define K1SIM_SHARED_SIM_FREEBODY_HPP

#include <array>
#include <cstring>
#include <mujoco/mujoco.h>

// Kinematics of a free-jointed body at one of its geoms' centres, read from and written to qpos/qvel
// directly, so it is exact at the current state without needing mj_forward's derived quantities.
//
// Needed because the scene ball's sphere geom sits off its body origin (k1_scene_robocup.xml keeps
// the body at the world origin and offsets the geom; see that file's header). A free joint's qvel
// is the body ORIGIN's linear velocity (world frame) and the angular velocity in the BODY frame, so
// the geom centre moves at v_origin + w x (R * geom_pos), not v_origin, whenever the ball spins.
//
// Callers must hold the sim mutex, like every other mjData access outside the physics thread.
namespace k1sim::freebody {

    struct GeomCentreState {
        std::array<double, 3> position{};  // world frame (m)
        std::array<double, 3> lin_vel{};   // world frame (m/s)
        std::array<double, 3> ang_vel{};   // world frame (rad/s)
    };

    // body_id must own exactly one joint and it must be free; geom_id must belong to body_id.
    inline bool valid_free_body_geom(const mjModel* m, int body_id, int geom_id) {
        if (m == nullptr || body_id < 0 || body_id >= m->nbody || geom_id < 0 || geom_id >= m->ngeom) {
            return false;
        }
        if (m->geom_bodyid[geom_id] != body_id || m->body_jntnum[body_id] != 1) {
            return false;
        }
        return m->jnt_type[m->body_jntadr[body_id]] == mjJNT_FREE;
    }

    // World-frame offset from the body origin to the geom centre, R(q) * geom_pos.
    inline std::array<double, 3> geom_offset_world(const mjModel* m, const mjData* d, int body_id, int geom_id) {
        const int qadr = m->jnt_qposadr[m->body_jntadr[body_id]];
        mjtNum offset[3];
        mju_rotVecQuat(offset, m->geom_pos + 3 * geom_id, d->qpos + qadr + 3);
        return {offset[0], offset[1], offset[2]};
    }

    inline GeomCentreState geom_centre_state(const mjModel* m, const mjData* d, int body_id, int geom_id) {
        GeomCentreState s{};
        if (!valid_free_body_geom(m, body_id, geom_id)) {
            return s;
        }
        const int jnt  = m->body_jntadr[body_id];
        const int qadr = m->jnt_qposadr[jnt];
        const int vadr = m->jnt_dofadr[jnt];

        const auto offset = geom_offset_world(m, d, body_id, geom_id);
        mjtNum w_world[3];
        mju_rotVecQuat(w_world, d->qvel + vadr + 3, d->qpos + qadr + 3);
        const mjtNum r[3] = {offset[0], offset[1], offset[2]};
        mjtNum w_cross_r[3];
        mju_cross(w_cross_r, w_world, r);

        for (int k = 0; k < 3; ++k) {
            s.position[k] = d->qpos[qadr + k] + offset[k];
            s.lin_vel[k]  = d->qvel[vadr + k] + w_cross_r[k];
            s.ang_vel[k]  = w_world[k];
        }
        return s;
    }

    // Set the free body's qvel so the geom centre moves at lin_vel with world-frame spin ang_vel.
    inline bool set_geom_centre_velocity(const mjModel* m,
                                         mjData* d,
                                         int body_id,
                                         int geom_id,
                                         const std::array<double, 3>& lin_vel,
                                         const std::array<double, 3>& ang_vel) {
        if (d == nullptr || !valid_free_body_geom(m, body_id, geom_id)) {
            return false;
        }
        const int jnt  = m->body_jntadr[body_id];
        const int qadr = m->jnt_qposadr[jnt];
        const int vadr = m->jnt_dofadr[jnt];

        const auto offset  = geom_offset_world(m, d, body_id, geom_id);
        const mjtNum w[3]  = {ang_vel[0], ang_vel[1], ang_vel[2]};
        const mjtNum r[3]  = {offset[0], offset[1], offset[2]};
        mjtNum w_cross_r[3];
        mju_cross(w_cross_r, w, r);

        // Angular velocity into the body frame: R^T w
        mjtNum q_inv[4];
        mju_negQuat(q_inv, d->qpos + qadr + 3);
        mjtNum w_local[3];
        mju_rotVecQuat(w_local, w, q_inv);

        for (int k = 0; k < 3; ++k) {
            d->qvel[vadr + k]     = lin_vel[k] - w_cross_r[k];
            d->qvel[vadr + 3 + k] = w_local[k];
        }
        return true;
    }

    // Spin for rolling without slipping on a horizontal floor: the contact point, r below the
    // centre, must be at rest, so v + w x (0, 0, -r) = 0 for the horizontal part of v.
    inline std::array<double, 3> rolling_spin(const std::array<double, 3>& lin_vel, double radius) {
        if (radius <= 0.0) {
            return {0.0, 0.0, 0.0};
        }
        return {-lin_vel[1] / radius, lin_vel[0] / radius, 0.0};
    }


    // Moves a free body's origin onto one of its geoms, at the spec level (before compiling).
    //
    // Why: the scene ball's body sits at the world origin with its sphere geom 1.38 m away (so the
    // zero-padded inherited keyframes still put the ball on its spot). MuJoCo integrates a free
    // body's ORIGIN linearly and its rotation separately, so a spinning ball's centre drifts by
    // ~0.5 (w dt)^2 |offset| per step: at 3 m/s rolling that is millimetres per step, the ball digs
    // into the floor and the contact launches it (3 m/s became 68 m/s within 0.1 s). With the
    // origin on the geom the error vanishes and the ball rolls cleanly.
    //
    // Editing the spec rather than the compiled model keeps the body's collision bounding volumes,
    // built at compile time in the body frame, consistent with the moved geoms.
    //
    // Shifts every geom, site and child body of the body by the offset. Returns the offset applied
    // (zero, and nothing changed, if the body or geom is missing, the body has no single free joint,
    // or its frame is rotated). Pair with shift_reoriginated_keyframes() after compiling.
    inline std::array<double, 3> reorigin_body_at_geom(mjSpec* spec, const char* body_name, const char* geom_name) {
        const std::array<double, 3> none{0.0, 0.0, 0.0};
        mjsBody* body = mjs_findBody(spec, body_name);
        if (body == nullptr) {
            return none;
        }
        mjsElement* joint = mjs_firstChild(body, mjOBJ_JOINT, 0);
        if (joint == nullptr || mjs_nextChild(body, joint, 0) != nullptr || mjs_asJoint(joint)->type != mjJNT_FREE) {
            return none;
        }
        if (body->quat[0] != 1.0 || body->quat[1] != 0.0 || body->quat[2] != 0.0 || body->quat[3] != 0.0) {
            return none;
        }
        mjsGeom* target = nullptr;
        for (mjsElement* el = mjs_firstChild(body, mjOBJ_GEOM, 0); el != nullptr; el = mjs_nextChild(body, el, 0)) {
            if (std::strcmp(mjs_getString(mjs_getName(el)), geom_name) == 0) {
                target = mjs_asGeom(el);
                break;
            }
        }
        if (target == nullptr) {
            return none;
        }
        const std::array<double, 3> offset{target->pos[0], target->pos[1], target->pos[2]};
        if (offset == none) {
            return none;
        }

        for (int k = 0; k < 3; ++k) {
            body->pos[k] += offset[k];
            if (body->explicitinertial) {
                body->ipos[k] -= offset[k];
            }
        }
        for (mjsElement* el = mjs_firstChild(body, mjOBJ_GEOM, 0); el != nullptr; el = mjs_nextChild(body, el, 0)) {
            for (int k = 0; k < 3; ++k) {
                mjs_asGeom(el)->pos[k] -= offset[k];
            }
        }
        for (mjsElement* el = mjs_firstChild(body, mjOBJ_SITE, 0); el != nullptr; el = mjs_nextChild(body, el, 0)) {
            for (int k = 0; k < 3; ++k) {
                mjs_asSite(el)->pos[k] -= offset[k];
            }
        }
        for (mjsElement* el = mjs_firstChild(body, mjOBJ_BODY, 0); el != nullptr; el = mjs_nextChild(body, el, 0)) {
            for (int k = 0; k < 3; ++k) {
                mjs_asBody(el)->pos[k] -= offset[k];
            }
        }
        return offset;
    }

    // Keeps every keyframe placing the re-originated body's geom exactly where it was before
    // reorigin_body_at_geom(): a keyframe stored the old origin, so the new origin is that plus the
    // offset rotated by the keyframe's orientation. (The inherited keyframes zero-pad the ball, so
    // this puts it back on its spot instead of at the world origin inside the robot.)
    inline void shift_reoriginated_keyframes(mjModel* m, int body_id, const std::array<double, 3>& offset) {
        if (m == nullptr || body_id < 0 || body_id >= m->nbody || m->body_jntnum[body_id] != 1
            || (offset[0] == 0.0 && offset[1] == 0.0 && offset[2] == 0.0)) {
            return;
        }
        const int adr        = m->jnt_qposadr[m->body_jntadr[body_id]];
        const mjtNum off[3]  = {offset[0], offset[1], offset[2]};
        for (int key = 0; key < m->nkey; ++key) {
            mjtNum* q        = m->key_qpos + key * m->nq + adr;
            mjtNum quat[4]   = {q[3], q[4], q[5], q[6]};
            if (mju_normalize4(quat) < mjMINVAL) {
                quat[0] = 1.0;  // an all-zero (padded) quaternion reads as identity
            }
            mjtNum shift[3];
            mju_rotVecQuat(shift, off, quat);
            for (int k = 0; k < 3; ++k) {
                q[k] += shift[k];
            }
        }
    }

}  // namespace k1sim::freebody

#endif  // K1SIM_SHARED_SIM_FREEBODY_HPP
