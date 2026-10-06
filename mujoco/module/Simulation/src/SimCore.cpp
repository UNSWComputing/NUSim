#include "module/Simulation/src/SimCore.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <ctime>
#include <stdexcept>

#include "shared/k1/BoosterApi.hpp"
#include "shared/sim/FreeBody.hpp"
#include "shared/sim/HeadPose.hpp"
#include "shared/sim/RobotIdentity.hpp"
#include "shared/util/Config.hpp"

namespace k1sim {

    namespace {

        // ZYX (yaw-pitch-roll) Euler extraction from a wxyz quaternion — the conventional
        // roll/pitch/yaw decomposition used by the Booster wire format's imu_state.rpy.
        void quat_to_rpy(const std::array<double, 4>& q, std::array<double, 3>& rpy) {
            const double w = q[0];
            const double x = q[1];
            const double y = q[2];
            const double z = q[3];

            const double sinr_cosp = 2.0 * (w * x + y * z);
            const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
            rpy[0]                 = std::atan2(sinr_cosp, cosr_cosp);

            double sinp = 2.0 * (w * y - z * x);
            sinp        = std::clamp(sinp, -1.0, 1.0);
            rpy[1]      = std::asin(sinp);

            const double siny_cosp = 2.0 * (w * z + x * y);
            const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
            rpy[2]                 = std::atan2(siny_cosp, cosy_cosp);
        }

        double to_seconds(const timespec& t) {
            return static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_nsec) * 1e-9;
        }

        timespec add_seconds(timespec t, double seconds) {
            const double total_nsec = static_cast<double>(t.tv_nsec) + seconds * 1e9;
            auto sec_adjust         = static_cast<long long>(std::floor(total_nsec / 1e9));
            t.tv_sec += sec_adjust;
            t.tv_nsec = static_cast<long>(total_nsec - static_cast<double>(sec_adjust) * 1e9);
            return t;
        }

        void sleep_until_monotonic(const timespec& deadline) {
#if defined(__APPLE__)
            // macOS has clock_gettime/nanosleep, but no clock_nanosleep(TIMER_ABSTIME).
            // Recompute against the absolute deadline after interruptions so pacing cannot drift.
            while (true) {
                timespec now{};
                clock_gettime(CLOCK_MONOTONIC, &now);
                timespec remaining{deadline.tv_sec - now.tv_sec, deadline.tv_nsec - now.tv_nsec};
                if (remaining.tv_nsec < 0) {
                    --remaining.tv_sec;
                    remaining.tv_nsec += 1000000000L;
                }
                if (remaining.tv_sec < 0 || (remaining.tv_sec == 0 && remaining.tv_nsec == 0)) {
                    return;
                }
                if (nanosleep(&remaining, nullptr) == 0 || errno != EINTR) {
                    return;
                }
            }
#else
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
#endif
        }

        // Standing base height of the "ready" pose; game spawns and the extra_spawn grid use it.
        constexpr double STAND_Z = 0.555;

        // Spawn slot for the k-th extra robot (k starts at 1): a 5x4 grid on the field,
        // rows at y = +-1.2 / +-2.4 so nothing lands on the y=0 line the main robot and
        // ball spawn on. z is the "ready" standing base height. The scene keyframes
        // zero-pad the extra free joints (global coords — zeros mean "at the origin,
        // inside the main robot"), so these slots are written into qpos explicitly
        // after every keyframe reset rather than relying on the padding.
        std::array<double, 3> extra_spawn(int k) {
            const int col  = (k - 1) % 5;
            const int row  = (k - 1) / 5;
            const double x = -3.0 + 1.5 * col;
            const double y = (row % 2 == 0 ? 1.0 : -1.0) * (1.2 + 1.2 * (row / 2));
            return {x, y, STAND_Z};
        }

        // Build the scene through mjSpec: re-originate the ball body on its sphere (see
        // freebody::reorigin_body_at_geom) and attach (robots - 1) extra K1 copies. Each copy's
        // element names get a "subNN_" prefix, so the main robot's unprefixed names (and
        // every existing name-based lookup) stay valid. Extra copies' keyframes are dropped;
        // the parent scene's keyframes zero-pad the new free joints, landing each copy at
        // its attachment frame.
        // extras[k - 1] is the attachment position of copy k (empty for a single robot).
        mjModel* load_multi_robot_model(const std::string& scene_path,
                                        const std::vector<std::array<double, 3>>& extras,
                                        char* error,
                                        int error_sz) {
            mjSpec* scene = mj_parseXML(scene_path.c_str(), nullptr, error, error_sz);
            if (scene == nullptr) {
                throw std::runtime_error("mj_parseXML failed for '" + scene_path + "': " + error);
            }

            const std::string robot_xml = scene_path.substr(0, scene_path.find_last_of('/') + 1) + "K1_22dof.xml";

            mjsBody* world = mjs_findBody(scene, "world");
            for (int k = 1; k <= static_cast<int>(extras.size()); ++k) {
                mjSpec* robot = mj_parseXML(robot_xml.c_str(), nullptr, error, error_sz);
                if (robot == nullptr) {
                    mj_deleteSpec(scene);
                    throw std::runtime_error("mj_parseXML failed for '" + robot_xml + "': " + error);
                }
                // The copy inherits the scene keyframes' zero-padding; its own (robot-sized)
                // keyframes would collide with the scene's on attach, so drop them.
                for (mjsElement* key = mjs_firstElement(robot, mjOBJ_KEY); key != nullptr;
                     key             = mjs_firstElement(robot, mjOBJ_KEY)) {
                    mjs_delete(robot, key);
                }

                mjsFrame* frame      = mjs_addFrame(world, nullptr);
                const auto& pos      = extras[k - 1];
                frame->pos[0]        = pos[0];
                frame->pos[1]        = pos[1];
                frame->pos[2]        = pos[2];
                mjsBody* trunk       = mjs_findBody(robot, "Trunk");
                mjsElement* attached = mjs_attach(frame->element, trunk->element, robot_prefix(k).c_str(), "");
                if (attached == nullptr) {
                    const std::string what = std::string("mjs_attach failed for robot copy ") + std::to_string(k) + ": "
                                             + mjs_getError(scene);
                    mj_deleteSpec(robot);
                    mj_deleteSpec(scene);
                    throw std::runtime_error(what);
                }
                mj_deleteSpec(robot);
            }

            // Put the ball body's origin on its sphere (see freebody::reorigin_body_at_geom)
            const auto ball_offset = freebody::reorigin_body_at_geom(scene, "ball", "ball");

            mjModel* m = mj_compile(scene, nullptr);
            if (m == nullptr) {
                const std::string what = std::string("mj_compile failed for multi-robot scene: ") + mjs_getError(scene);
                mj_deleteSpec(scene);
                throw std::runtime_error(what);
            }
            mj_deleteSpec(scene);
            freebody::shift_reoriginated_keyframes(m, mj_name2id(m, mjOBJ_BODY, "ball"), ball_offset);
            return m;
        }

    }  // namespace

    SimCore::SimCore(Config config, StateCallback on_state)
        : config_(std::move(config)), on_state_(std::move(on_state)) {
        pd_.kp = config_.kp;
        pd_.kd = config_.kd;
    }

    SimCore::~SimCore() {
        stop();
        unload();
    }

    void SimCore::load_model() {
        const std::string resolved = config::resolve_path(config_.model_path).string();

        char error[1024] = {0};
        // Always through the spec (no extras for a single robot), so the ball is re-originated
        std::vector<std::array<double, 3>> extras;
        for (int k = 1; k < config_.robots; ++k) {
            const auto pose = spawn_pose(k);
            extras.push_back({pose[0], pose[1], pose[2]});
        }
        m_ = load_multi_robot_model(resolved, extras, error, sizeof(error));

        // Throws if any joint/actuator is missing or there is no free root joint.
        map_ = ModelMap::build(m_);

        apply_surface_override();

        left_foot_body_id_  = mj_name2id(m_, mjOBJ_BODY, "left_foot_link");
        right_foot_body_id_ = mj_name2id(m_, mjOBJ_BODY, "right_foot_link");

        ball_body_id_ = mj_name2id(m_, mjOBJ_BODY, "ball");
        ball_geom_id_ = mj_name2id(m_, mjOBJ_GEOM, "ball");
        if (!freebody::valid_free_body_geom(m_, ball_body_id_, ball_geom_id_)) {
            ball_body_id_ = ball_geom_id_ = -1;
            std::fprintf(stderr, "SimCore: scene has no free \"ball\" body/geom; ball ground truth is off\n");
        }

        head_body_id_ = mj_name2id(m_, mjOBJ_BODY, "Head_2");
        if (head_body_id_ < 0) {
            std::fprintf(stderr, "SimCore: model has no Head_2 body; rt/head_pose will not be published\n");
        }
        if (!config_.foot_log_path.empty()) {
            if (left_foot_body_id_ < 0 || right_foot_body_id_ < 0) {
                std::fprintf(stderr, "SimCore: foot log requested but the model has no left/right_foot_link\n");
            }
            else {
                foot_log_ = std::fopen(config_.foot_log_path.c_str(), "w");
                if (foot_log_ == nullptr) {
                    std::fprintf(stderr, "SimCore: could not open foot log '%s'\n", config_.foot_log_path.c_str());
                }
                else {
                    std::fprintf(foot_log_,
                                 "t,l_fz,l_cop_x,l_cop_y,l_pitch,l_roll,"
                                 "r_fz,r_cop_x,r_cop_y,r_pitch,r_roll,base_pitch,base_z\n");
                    std::fprintf(stderr, "SimCore: foot contact log -> %s\n", config_.foot_log_path.c_str());
                }
            }
        }

        // Index maps for the extra robot copies so the physics loop can PD-hold them
        // upright. Only the three per-joint index arrays are needed.
        extra_maps_.clear();
        for (int k = 1; k < config_.robots; ++k) {
            const std::string prefix = robot_prefix(k);
            ModelMap em{};
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                const std::string name = prefix + JOINT_NAMES[i];
                const int jnt          = mj_name2id(m_, mjOBJ_JOINT, name.c_str());
                const int act          = mj_name2id(m_, mjOBJ_ACTUATOR, name.c_str());
                if (jnt < 0 || act < 0) {
                    throw std::runtime_error("multi-robot scene is missing joint/actuator '" + name + "'");
                }
                em.qpos_adr[i] = m_->jnt_qposadr[jnt];
                em.dof_adr[i]  = m_->jnt_dofadr[jnt];
                em.act_id[i]   = act;
            }
            const std::string root = prefix + "root";
            const int root_jnt     = mj_name2id(m_, mjOBJ_JOINT, root.c_str());
            if (root_jnt < 0) {
                throw std::runtime_error("multi-robot scene is missing free joint '" + root + "'");
            }
            em.root_qpos_adr = m_->jnt_qposadr[root_jnt];
            em.root_dof_adr  = m_->jnt_dofadr[root_jnt];
            extra_maps_.push_back(em);
        }

        d_ = mj_makeData(m_);
        if (d_ == nullptr) {
            mj_deleteModel(m_);
            m_ = nullptr;
            throw std::runtime_error("mj_makeData failed for '" + resolved + "'");
        }

        // Spawn keyframe (configurable, e.g. lying_front for get-up testing); the PD
        // fallback target always tracks the "ready" pose regardless of where we spawn.
        int spawn_key = mj_name2id(m_, mjOBJ_KEY, config_.initial_keyframe.c_str());
        if (spawn_key < 0 && config_.initial_keyframe != "ready") {
            std::fprintf(stderr,
                         "SimCore: model has no keyframe '%s'; falling back to 'ready'\n",
                         config_.initial_keyframe.c_str());
            spawn_key = mj_name2id(m_, mjOBJ_KEY, "ready");
        }
        reset_key_ = spawn_key;
        if (spawn_key >= 0) {
            mj_resetDataKeyframe(m_, d_, spawn_key);
        }

        const int ready_key = mj_name2id(m_, mjOBJ_KEY, "ready");
        if (ready_key >= 0) {
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                ready_target_[i] = m_->key_qpos[ready_key * m_->nq + map_.qpos_adr[i]];
            }
        }
        else {
            ready_target_ = config_.ready_pose_fallback;
        }

        place_robots();

        // Populate derived quantities (xquat, sensordata, ...) for the reset pose before the
        // physics thread's first mj_step; harmless if nothing reads them this early.
        mj_forward(m_, d_);
    }

    // Make the floor's declared contact parameters the ones the feet actually see.
    //
    // MuJoCo derives a contact's parameters from the two geoms: if their priorities are equal
    // it takes the element-wise MAX of the friction vectors. The K1 foot box carries no
    // explicit friction, so it gets the MuJoCo default of 1.0, and max(1.0, floor) means the
    // floor's number has never mattered -- a "0.8 grass" scene has been simulating mu = 1.0.
    // Raising the floor's priority makes it authoritative, which is what the mujoco_playground
    // training scene does (its floor geom is declared priority="1").
    //
    // Explicit <pair> elements (the scene defines one for ball-vs-floor) are unaffected by
    // priority, so tuned ball dynamics survive this.
    void SimCore::apply_surface_override() {
        if (!config_.surface.enabled) {
            return;
        }
        const int floor = mj_name2id(m_, mjOBJ_GEOM, "floor");
        if (floor < 0) {
            std::fprintf(stderr, "SimCore: surface override requested but the scene has no geom 'floor'\n");
            return;
        }

        const double min_timeconst = 2.0 * m_->opt.timestep;
        double timeconst           = config_.surface.solref_timeconst;
        if (timeconst < min_timeconst) {
            std::fprintf(stderr,
                         "SimCore: surface.solref_timeconst %g is below 2*timestep (%g); clamping\n",
                         timeconst,
                         min_timeconst);
            timeconst = min_timeconst;
        }

        m_->geom_priority[floor]       = 1;
        m_->geom_friction[3 * floor]   = config_.surface.friction;
        m_->geom_solref[2 * floor]     = timeconst;
        m_->geom_solref[2 * floor + 1] = config_.surface.solref_dampratio;

        std::fprintf(stderr,
                     "SimCore: floor contact overridden — mu = %g, solref = [%g, %g], priority = 1\n",
                     config_.surface.friction,
                     timeconst,
                     config_.surface.solref_dampratio);
    }

    // One CSV row per published state: per foot the total contact normal force, the centre of
    // pressure expressed in that foot's own frame, and the sole's pitch/roll.
    //
    // The centre of pressure is what makes tiptoe measurable rather than a description of a
    // video. The sole box spans x in [-0.064, +0.116] of the foot frame, so a flat-footed
    // stance sits near cop_x = 0.026 (the box centre) and a foot rolled onto its toe pushes
    // cop_x towards +0.116 -- the front edge of the support polygon, where the available
    // friction is spent on a shrinking contact patch. Caller holds mutex_.
    void SimCore::log_foot_state() {
        if (foot_log_ == nullptr) {
            return;
        }

        struct FootAcc {
            double fz    = 0.0;  // summed normal force
            double cop_x = 0.0;  // force-weighted, foot frame
            double cop_y = 0.0;
            double pitch = 0.0;  // sole tilt, world
            double roll  = 0.0;
        };
        std::array<FootAcc, 2> feet{};
        const std::array<int, 2> body_ids{left_foot_body_id_, right_foot_body_id_};

        for (int f = 0; f < 2; ++f) {
            if (body_ids[f] < 0) {
                continue;
            }
            // The foot's own +z axis expressed in world; its x/y components are the sole tilt.
            const mjtNum* R = d_->xmat + 9 * body_ids[f];
            feet[f].pitch   = std::atan2(R[0 * 3 + 2], R[2 * 3 + 2]);
            feet[f].roll    = std::atan2(R[1 * 3 + 2], R[2 * 3 + 2]);
        }

        for (int c = 0; c < d_->ncon; ++c) {
            const mjContact& con = d_->contact[c];
            const int body1      = m_->geom_bodyid[con.geom1];
            const int body2      = m_->geom_bodyid[con.geom2];
            int f                = -1;
            if (body1 == left_foot_body_id_ || body2 == left_foot_body_id_) {
                f = 0;
            }
            else if (body1 == right_foot_body_id_ || body2 == right_foot_body_id_) {
                f = 1;
            }
            if (f < 0) {
                continue;
            }

            // force[0] is the normal component in the contact frame.
            mjtNum force[6] = {0};
            mj_contactForce(m_, d_, c, force);
            const double fn = force[0];
            if (fn <= 0.0) {
                continue;
            }

            // Contact point into the foot's frame: local = R^T * (pos - body_pos).
            const mjtNum* R     = d_->xmat + 9 * body_ids[f];
            const mjtNum* org   = d_->xpos + 3 * body_ids[f];
            const mjtNum rel[3] = {con.pos[0] - org[0], con.pos[1] - org[1], con.pos[2] - org[2]};
            mjtNum local[3];
            mju_mulMatTVec3(local, R, rel);

            feet[f].fz += fn;
            feet[f].cop_x += fn * local[0];
            feet[f].cop_y += fn * local[1];
        }

        for (auto& foot : feet) {
            if (foot.fz > 0.0) {
                foot.cop_x /= foot.fz;
                foot.cop_y /= foot.fz;
            }
        }

        std::array<double, 4> quat{1, 0, 0, 0};
        if (map_.root_body_id >= 0) {
            for (int k = 0; k < 4; ++k) {
                quat[k] = d_->xquat[4 * map_.root_body_id + k];
            }
        }
        std::array<double, 3> rpy{};
        quat_to_rpy(quat, rpy);

        std::fprintf(foot_log_,
                     "%.4f,%.3f,%.5f,%.5f,%.5f,%.5f,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                     d_->time,
                     feet[0].fz,
                     feet[0].cop_x,
                     feet[0].cop_y,
                     feet[0].pitch,
                     feet[0].roll,
                     feet[1].fz,
                     feet[1].cop_x,
                     feet[1].cop_y,
                     feet[1].pitch,
                     feet[1].roll,
                     rpy[1],
                     d_->qpos[map_.root_qpos_adr + 2]);
    }

    // Put every extra robot copy at its spawn slot in the ready pose with zero velocity.
    // Keyframe resets zero-pad the extras' free joints (= world origin, inside the main
    // robot), so this must run after every keyframe reset. Caller holds mutex_ (or the
    // physics thread is not running yet).
    std::array<double, 4> SimCore::spawn_pose(int k) const {
        if (!config_.spawns.empty()) {
            const auto& s = config_.spawns[k];
            return {s[0], s[1], STAND_Z, s[2]};
        }
        const auto pos = extra_spawn(k);
        return {pos[0], pos[1], pos[2], 0.0};
    }

    void SimCore::place_robots() {
        // A game places the main robot too: its keyframe keeps the joint pose, base height and
        // tilt (so e.g. lying_front still spawns lying down), with the base moved to the game's
        // x/y and turned by its yaw.
        if (!config_.spawns.empty()) {
            const auto pose = spawn_pose(0);
            double* root    = d_->qpos + map_.root_qpos_adr;
            const double cw = std::cos(pose[3] / 2.0);
            const double sw = std::sin(pose[3] / 2.0);
            const std::array<double, 4> q{root[3], root[4], root[5], root[6]};
            root[0] = pose[0];
            root[1] = pose[1];
            root[3] = cw * q[0] - sw * q[3];  // (cw, 0, 0, sw) * q, wxyz
            root[4] = cw * q[1] - sw * q[2];
            root[5] = cw * q[2] + sw * q[1];
            root[6] = cw * q[3] + sw * q[0];
            place_ball_on_centre_mark();
        }

        for (std::size_t j = 0; j < extra_maps_.size(); ++j) {
            const ModelMap& em             = extra_maps_[j];
            const auto pose                = spawn_pose(static_cast<int>(j) + 1);
            d_->qpos[em.root_qpos_adr + 0] = pose[0];
            d_->qpos[em.root_qpos_adr + 1] = pose[1];
            d_->qpos[em.root_qpos_adr + 2] = pose[2];
            d_->qpos[em.root_qpos_adr + 3] = std::cos(pose[3] / 2.0);  // yaw-only quat (w,x,y,z)
            d_->qpos[em.root_qpos_adr + 4] = 0.0;
            d_->qpos[em.root_qpos_adr + 5] = 0.0;
            d_->qpos[em.root_qpos_adr + 6] = std::sin(pose[3] / 2.0);
            for (int v = 0; v < 6; ++v) {
                d_->qvel[em.root_dof_adr + v] = 0.0;
            }
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                d_->qpos[em.qpos_adr[i]] = ready_target_[i];
                d_->qvel[em.dof_adr[i]]  = 0.0;
            }
        }
    }

    void SimCore::place_ball_on_centre_mark() {
        // Find the ball, if the scene has one
        const int joint = mj_name2id(m_, mjOBJ_JOINT, "ball_free");
        const int geom  = mj_name2id(m_, mjOBJ_GEOM, "ball");
        if (joint < 0 || geom < 0) {
            return;
        }
        // Move the ball back by its x offset so it sits on the centre mark
        d_->qpos[m_->jnt_qposadr[joint]] = -m_->geom_pos[3 * geom];
    }

    void SimCore::reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (reset_key_ >= 0) {
            mj_resetDataKeyframe(m_, d_, reset_key_);
        }
        else {
            mj_resetData(m_, d_);
        }
        place_robots();
        // Repopulate derived quantities so snapshots/viewer frames between now and the next
        // mj_step see the reset pose, not stale kinematics.
        mj_forward(m_, d_);
    }

    void SimCore::start() {
        bool expected = false;
        if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            return;  // already running
        }
        thread_ = std::thread(&SimCore::physics_loop, this);
    }

    void SimCore::stop() {
        running_.store(false, std::memory_order_release);
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    void SimCore::unload() {
        if (foot_log_ != nullptr) {
            std::fclose(foot_log_);
            foot_log_ = nullptr;
        }
        if (d_ != nullptr) {
            mj_deleteData(d_);
            d_ = nullptr;
        }
        if (m_ != nullptr) {
            mj_deleteModel(m_);
            m_ = nullptr;
        }
    }

    std::unique_ptr<message::SimStateUpdate> SimCore::make_snapshot(uint64_t steps) const {
        auto s        = std::make_unique<message::SimStateUpdate>();
        s->sim_time   = d_->time;
        s->wall_time_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
        s->step_count = steps;

        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            auto& j = s->joints[i];
            j.q     = d_->qpos[map_.qpos_adr[i]];
            j.dq    = d_->qvel[map_.dof_adr[i]];
            j.ddq   = d_->qacc[map_.dof_adr[i]];
            j.tau   = d_->actuator_force[map_.act_id[i]];
        }

        // IMU orientation + rpy.
        std::array<double, 4> quat{1, 0, 0, 0};
        if (map_.sens_quat >= 0) {
            for (int k = 0; k < 4; ++k) {
                quat[k] = d_->sensordata[map_.sens_quat + k];
            }
        }
        else if (map_.root_body_id >= 0) {
            for (int k = 0; k < 4; ++k) {
                quat[k] = d_->xquat[4 * map_.root_body_id + k];
            }
        }
        s->imu.quat = quat;
        quat_to_rpy(quat, s->imu.rpy);

        if (map_.sens_gyro >= 0) {
            for (int k = 0; k < 3; ++k) {
                s->imu.gyro[k] = d_->sensordata[map_.sens_gyro + k];
            }
        }
        else if (map_.root_body_id >= 0) {
            // Fallback: cvel's angular part is world-axis-aligned; rotate into the body's local
            // frame with the body rotation matrix (R^T * world = mju_mulMatTVec3).
            const mjtNum* rmat    = d_->xmat + 9 * map_.root_body_id;
            const mjtNum world[3] = {d_->cvel[6 * map_.root_body_id + 0],
                                     d_->cvel[6 * map_.root_body_id + 1],
                                     d_->cvel[6 * map_.root_body_id + 2]};
            mjtNum local[3];
            mju_mulMatTVec3(local, rmat, world);
            s->imu.gyro = {local[0], local[1], local[2]};
        }

        if (map_.sens_acc >= 0) {
            for (int k = 0; k < 3; ++k) {
                s->imu.acc[k] = d_->sensordata[map_.sens_acc + k];
            }
        }
        else if (map_.root_body_id >= 0) {
            // Fallback approximation: the reading a stationary accelerometer would show under
            // gravity alone (R^T * (0,0,+g)); ignores true linear acceleration (no cacc bookkeeping
            // without the real sensor). Good enough for a defensive path that isn't exercised by
            // the vendored model, which always carries the real sensor.
            const mjtNum* rmat    = d_->xmat + 9 * map_.root_body_id;
            const mjtNum g        = -m_->opt.gravity[2];
            const mjtNum world[3] = {0, 0, g};
            mjtNum local[3];
            mju_mulMatTVec3(local, rmat, world);
            s->imu.acc = {local[0], local[1], local[2]};
        }

        // Base pose/velocity (free root joint). qpos: [x y z qw qx qy qz]; qvel: [vx vy vz wx wy wz]
        // with the linear part in world frame and the angular part in the body's local frame.
        const int qadr  = map_.root_qpos_adr;
        const int vadr  = map_.root_dof_adr;
        s->base.x       = d_->qpos[qadr + 0];
        s->base.y       = d_->qpos[qadr + 1];
        s->base.z       = d_->qpos[qadr + 2];
        s->base.quat    = {d_->qpos[qadr + 3], d_->qpos[qadr + 4], d_->qpos[qadr + 5], d_->qpos[qadr + 6]};
        s->base.lin_vel = {d_->qvel[vadr + 0], d_->qvel[vadr + 1], d_->qvel[vadr + 2]};
        {
            const mjtNum* rmat    = d_->xmat + 9 * map_.root_body_id;
            const mjtNum local[3] = {d_->qvel[vadr + 3], d_->qvel[vadr + 4], d_->qvel[vadr + 5]};
            mjtNum world[3];
            mju_mulMatVec3(world, rmat, local);
            s->base.ang_vel = {world[0], world[1], world[2]};
        }

        if (head_body_id_ >= 0) {
            const FootprintPose Hrh = head_in_footprint(d_->xpos + 3 * head_body_id_,
                                                        d_->xquat + 4 * head_body_id_,
                                                        d_->qpos + qadr,
                                                        d_->qpos + qadr + 3);
            s->head.valid           = true;
            s->head.position        = Hrh.position;
            s->head.quat            = Hrh.quat;
        }

        if (ball_body_id_ >= 0) {
            const auto ball    = freebody::geom_centre_state(m_, d_, ball_body_id_, ball_geom_id_);
            s->ball.valid      = true;
            s->ball.position   = ball.position;
            s->ball.lin_vel    = ball.lin_vel;
            s->ball.ang_vel    = ball.ang_vel;
        }

        StepController* ctrl = controller_.load(std::memory_order_acquire);
        if (ctrl != nullptr) {
            s->mode       = ctrl->mode();
            s->fall_state = ctrl->fall_state();
            s->getting_up = ctrl->getting_up();
        }
        else {
            s->mode       = booster::RobotMode::PREPARE;
            s->fall_state = booster::FallState::IS_READY;
            s->getting_up = false;
        }

        s->measured_rtf = measured_rtf_.load(std::memory_order_relaxed);
        return s;
    }

    void SimCore::physics_loop() {
        const double dt     = m_->opt.timestep;
        const bool free_run = config_.rtf <= 0.0;
        const double period = free_run ? 0.0 : dt / config_.rtf;
        const auto publish_every =
            static_cast<uint64_t>(config_.state_publish_divisor > 0 ? config_.state_publish_divisor : 0);

        timespec deadline{};
        clock_gettime(CLOCK_MONOTONIC, &deadline);

        double window_wall_start   = to_seconds(deadline);
        uint64_t window_step_start = 0;
        uint64_t steps             = 0;

        while (running_.load(std::memory_order_acquire)) {
            std::unique_ptr<message::SimStateUpdate> snapshot;
            {
                std::lock_guard<std::mutex> lock(mutex_);

                StepController* ctrl = controller_.load(std::memory_order_acquire);
                if (ctrl != nullptr) {
                    ctrl->step(m_, d_);
                }
                else {
                    pd_.apply(m_, d_, map_, ready_target_);
                }
                // Extra --robots copies have no controller; hold them at the ready pose.
                for (const auto& em : extra_maps_) {
                    pd_.apply(m_, d_, em, ready_target_);
                }
                mj_step(m_, d_);
                ++steps;
                step_count_.store(steps, std::memory_order_relaxed);

                if (publish_every > 0 && steps % publish_every == 0) {
                    snapshot = make_snapshot(steps);
                    log_foot_state();
                }
            }  // release the mutex before emitting/pacing

            if (snapshot && on_state_) {
                on_state_(std::move(snapshot));
            }

            if (!free_run) {
                deadline = add_seconds(deadline, period);
                timespec now_ts{};
                clock_gettime(CLOCK_MONOTONIC, &now_ts);
                const double behind = to_seconds(now_ts) - to_seconds(deadline);
                if (behind > config_.resync_threshold) {
                    deadline = now_ts;
                    dropped_deadlines_.fetch_add(1, std::memory_order_relaxed);
                }
                sleep_until_monotonic(deadline);
            }

            timespec wall_now{};
            clock_gettime(CLOCK_MONOTONIC, &wall_now);
            const double wall_elapsed = to_seconds(wall_now) - window_wall_start;
            if (wall_elapsed >= 1.0) {
                const double sim_elapsed = static_cast<double>(steps - window_step_start) * dt;
                measured_rtf_.store(sim_elapsed / wall_elapsed, std::memory_order_relaxed);
                window_wall_start = to_seconds(wall_now);
                window_step_start = steps;
            }
        }
    }

}  // namespace k1sim
