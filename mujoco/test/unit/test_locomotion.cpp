// Headless, free-run unit tests for module::Locomotion's reduced mode state
// machine (LocomotionController: Damping / Prepare / Custom). No NUClear/
// PowerPlant involved: LocomotionController is deliberately framework-free
// (see its header), so this test loads the model directly and drives
// controller.step() + mj_step() itself, exactly as module::Simulation's
// physics thread will in the real binary (StepController contract: step()
// runs every physics tick, immediately before mj_step()).
//
// Locomotion *policies* (walking, get-up, kick) live in NUbots_K1 and reach
// the sim as LowCmd servo targets in CUSTOM mode, so the tests here cover the
// servo-listener surface only: the Prepare blend/hold, CUSTOM LowCmd
// tracking, fall detection, and the wire-compat mapping of WALKING/SOCCER
// mode requests onto PREPARE.
//
// The model under test is the real vendored models/k1/K1_22dof.xml, NOT a
// copy -- a floor plane is added programmatically via MuJoCo's mjSpec
// model-editing API (mj_parseXML -> mjs_addGeom -> mj_compile) rather than an
// XML <include>, because MuJoCo resolves an included file's own
// <compiler meshdir="..."/> against the *outermost* file's directory, not
// the included file's (verified empirically while building this test) --
// mjSpec editing sidesteps that entirely and needs no companion XML fixture.
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "module/Locomotion/src/LocoMath.hpp"
#include "module/Locomotion/src/LocomotionController.hpp"
#include "shared/k1/BoosterApi.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/message/Commands.hpp"
#include "shared/sim/ModelMap.hpp"
#include "shared/util/Config.hpp"

using k1sim::JOINT_COUNT;
using k1sim::JointIndexK1;
using k1sim::ModelMap;
using k1sim::module::LocomotionController;
namespace booster = k1sim::booster;

namespace {

    int g_checks   = 0;
    int g_failures = 0;

    void check(bool cond, const std::string& msg) {
        ++g_checks;
        if (cond) {
            std::printf("  PASS: %s\n", msg.c_str());
        }
        else {
            std::printf("  FAIL: %s\n", msg.c_str());
            ++g_failures;
        }
    }

    double rad2deg(double rad) {
        return rad * 180.0 / 3.14159265358979323846;
    }

    // Loads the real vendored K1 model and adds a floor plane via the mjSpec
    // model-editing API (see the file header comment for why not an XML scene).
    mjModel* load_test_model() {
        const auto path  = k1sim::config::resolve_path("models/k1/K1_22dof.xml");
        char error[1024] = {0};
        mjSpec* spec     = mj_parseXML(path.string().c_str(), nullptr, error, sizeof(error));
        if (!spec) {
            std::fprintf(stderr, "test_locomotion: failed to parse %s: %s\n", path.string().c_str(), error);
            std::exit(1);
        }

        mjsBody* world     = mjs_findBody(spec, "world");
        mjsGeom* floor     = mjs_addGeom(world, nullptr);
        floor->type        = mjGEOM_PLANE;
        floor->size[0]     = 0.0;
        floor->size[1]     = 0.0;
        floor->size[2]     = 0.05;
        floor->friction[0] = 0.8;
        floor->friction[1] = 0.005;
        floor->friction[2] = 0.0001;

        mjModel* m = mj_compile(spec, nullptr);
        if (!m) {
            std::fprintf(stderr, "test_locomotion: failed to compile model: %s\n", mjs_getError(spec));
            mj_deleteSpec(spec);
            std::exit(1);
        }
        mj_deleteSpec(spec);
        return m;
    }

    void reset_to_keyframe(const mjModel* m, mjData* d, const char* keyframe) {
        const int key = mj_name2id(m, mjOBJ_KEY, keyframe);
        if (key < 0) {
            std::fprintf(stderr, "test_locomotion: model has no keyframe '%s'\n", keyframe);
            std::exit(1);
        }
        mj_resetDataKeyframe(m, d, key);
        mj_forward(m, d);
    }

    YAML::Node locomotion_cfg() {
        return k1sim::config::load("locomotion.yaml");
    }
    YAML::Node gains_cfg() {
        return k1sim::config::load("gains.yaml");
    }

    struct BaseState {
        double x, y, z;
        double tilt_rad;
    };

    BaseState read_base_state(mjData* d, const ModelMap& map) {
        BaseState s;
        s.x        = d->qpos[map.root_qpos_adr + 0];
        s.y        = d->qpos[map.root_qpos_adr + 1];
        s.z        = d->qpos[map.root_qpos_adr + 2];
        s.tilt_rad = k1sim::module::base_tilt(d, map);
        return s;
    }

    bool all_finite(const mjData* d, int nq) {
        for (int i = 0; i < nq; ++i) {
            if (!std::isfinite(d->qpos[i])) {
                return false;
            }
        }
        return true;
    }

    // Builds a full-body SERIAL LowCmd targeting `q_ref` with the given gains.
    std::vector<k1sim::message::MotorCmdData> make_low_cmd(const std::array<double, JOINT_COUNT>& q_ref,
                                                           const std::array<double, JOINT_COUNT>& kp,
                                                           const std::array<double, JOINT_COUNT>& kd) {
        std::vector<k1sim::message::MotorCmdData> motors(JOINT_COUNT);
        for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
            motors[j].mode = 1;
            motors[j].q    = static_cast<float>(q_ref[j]);
            motors[j].kp   = static_cast<float>(kp[j]);
            motors[j].kd   = static_cast<float>(kd[j]);
        }
        return motors;
    }

    std::array<double, JOINT_COUNT> load_joint_array(const YAML::Node& node) {
        std::array<double, JOINT_COUNT> arr{};
        const auto values = node.as<std::vector<double>>();
        for (std::size_t i = 0; i < JOINT_COUNT && i < values.size(); ++i) {
            arr[i] = values[i];
        }
        return arr;
    }

    constexpr double kStandHeightMin = 0.45;
    constexpr double kStandHeightMax = 0.62;

    // ---------------------------------------------------------------------
    // Test 1: PREPARE from the ready keyframe -- 10s, height/tilt held.
    // ---------------------------------------------------------------------
    void test_prepare() {
        std::printf("test_prepare:\n");
        mjModel* m = load_test_model();
        mjData* d  = mj_makeData(m);
        reset_to_keyframe(m, d, "ready");

        LocomotionController controller(locomotion_cfg(), gains_cfg());
        controller.request_mode_change(booster::PREPARE);

        const ModelMap map = ModelMap::build(m);
        for (int i = 0; i < 10000; ++i) {  // 10 s @ 1 kHz
            controller.step(m, d);
            mj_step(m, d);
        }

        const BaseState s = read_base_state(d, map);
        check(s.z > kStandHeightMin && s.z < kStandHeightMax,
              "base height in [0.45,0.62] (z=" + std::to_string(s.z) + ")");
        check(rad2deg(s.tilt_rad) < 10.0, "tilt < 10 deg (tilt=" + std::to_string(rad2deg(s.tilt_rad)) + " deg)");
        check(controller.mode() == booster::PREPARE, "mode() reports PREPARE");
        check(controller.fall_state() == booster::IS_READY, "fall_state()==IS_READY while standing");

        mj_deleteData(d);
        mj_deleteModel(m);
    }

    // ---------------------------------------------------------------------
    // Test 2: Head tracking (RotateHead) in PREPARE.
    // ---------------------------------------------------------------------
    void test_head() {
        std::printf("test_head:\n");
        mjModel* m = load_test_model();
        mjData* d  = mj_makeData(m);
        reset_to_keyframe(m, d, "ready");

        LocomotionController controller(locomotion_cfg(), gains_cfg());
        controller.request_mode_change(booster::PREPARE);
        controller.set_head_command(/*pitch=*/0.3, /*yaw=*/0.4);

        const ModelMap map = ModelMap::build(m);
        for (int i = 0; i < 1000; ++i) {  // 1 s @ 1 kHz
            controller.step(m, d);
            mj_step(m, d);
        }

        // Tolerances allow for PD steady-state error under gravity: the servo listener
        // applies no gravity feed-forward (that used to be the locomotion backends' job;
        // a NUbots_K1 policy compensates through its LowCmd tau/kp instead).
        const double head_pitch = d->qpos[map.qpos_adr[JointIndexK1::HeadPitch]];
        const double head_yaw   = d->qpos[map.qpos_adr[JointIndexK1::HeadYaw]];
        check(std::abs(head_pitch - 0.3) < 0.12,
              "Head_pitch reaches 0.3 within 1s (q=" + std::to_string(head_pitch) + ")");
        check(std::abs(head_yaw - 0.4) < 0.12, "AAHead_yaw reaches 0.4 within 1s (q=" + std::to_string(head_yaw) + ")");

        mj_deleteData(d);
        mj_deleteModel(m);
    }

    // ---------------------------------------------------------------------
    // Test 3: CUSTOM mode PD-tracks LowCmd servo targets (the path every
    // NUbots_K1 locomotion policy now uses).
    // ---------------------------------------------------------------------
    void test_custom_low_cmd() {
        std::printf("test_custom_low_cmd:\n");
        mjModel* m = load_test_model();
        mjData* d  = mj_makeData(m);
        reset_to_keyframe(m, d, "ready");

        const YAML::Node gains = gains_cfg();
        const auto ready_pose  = load_joint_array(gains["ready_pose"]);
        const auto kp          = load_joint_array(gains["kp"]);
        const auto kd          = load_joint_array(gains["kd"]);

        LocomotionController controller(locomotion_cfg(), gains_cfg());
        controller.request_mode_change(booster::CUSTOM);

        const ModelMap map = ModelMap::build(m);

        // Entering CUSTOM clears any stale LowCmd and PD-holds the entry pose until the
        // first command of the session arrives (dead-client protection), so let the mode
        // change land before streaming — like a real 50 Hz client.
        for (int i = 0; i < 100; ++i) {
            controller.step(m, d);
            mj_step(m, d);
        }

        // Target: ready pose with a deliberate offset on a few joints.
        std::array<double, JOINT_COUNT> q_ref  = ready_pose;
        q_ref[JointIndexK1::HeadYaw]           = 0.3;
        q_ref[JointIndexK1::LeftShoulderPitch] = 0.4;
        q_ref[JointIndexK1::RightElbowPitch]   = -0.5;
        controller.set_low_cmd(/*SERIAL=*/1, make_low_cmd(q_ref, kp, kd));

        for (int i = 0; i < 3000; ++i) {  // 3 s @ 1 kHz
            controller.step(m, d);
            mj_step(m, d);
        }

        check(controller.mode() == booster::CUSTOM, "mode() reports CUSTOM");
        const double q_head  = d->qpos[map.qpos_adr[JointIndexK1::HeadYaw]];
        const double q_shoul = d->qpos[map.qpos_adr[JointIndexK1::LeftShoulderPitch]];
        const double q_elbow = d->qpos[map.qpos_adr[JointIndexK1::RightElbowPitch]];
        // 0.12 tolerance: pure PD against gravity has steady-state error (the LowCmd here
        // sends no tau feed-forward; a real policy compensates through training).
        check(std::abs(q_head - 0.3) < 0.12, "LowCmd head yaw target tracked (q=" + std::to_string(q_head) + ")");
        check(std::abs(q_shoul - 0.4) < 0.12,
              "LowCmd shoulder pitch target tracked (q=" + std::to_string(q_shoul) + ")");
        check(std::abs(q_elbow + 0.5) < 0.12, "LowCmd elbow target tracked (q=" + std::to_string(q_elbow) + ")");

        const BaseState s = read_base_state(d, map);
        check(s.z > kStandHeightMin && s.z < kStandHeightMax,
              "still standing under LowCmd control (z=" + std::to_string(s.z) + ")");
        check(all_finite(d, m->nq), "qpos stays finite throughout");

        // An undersized PARALLEL command must not blow up (warn + hold behaviour).
        controller.set_low_cmd(/*PARALLEL=*/0, {});
        for (int i = 0; i < 500; ++i) {
            controller.step(m, d);
            mj_step(m, d);
        }
        check(all_finite(d, m->nq), "PARALLEL LowCmd: qpos stays finite (hold behaviour)");

        mj_deleteData(d);
        mj_deleteModel(m);
    }

    // ---------------------------------------------------------------------
    // Test 4: Fall detection from the lying_front keyframe.
    // ---------------------------------------------------------------------
    void test_fall_detection() {
        std::printf("test_fall_detection:\n");
        mjModel* m = load_test_model();
        mjData* d  = mj_makeData(m);
        reset_to_keyframe(m, d, "lying_front");

        LocomotionController controller(locomotion_cfg(), gains_cfg());
        controller.request_mode_change(booster::DAMPING);

        for (int i = 0; i < 1000; ++i) {  // 1 s to settle flat
            controller.step(m, d);
            mj_step(m, d);
        }

        check(controller.fall_state() == booster::HAS_FALLEN, "fall_state()==HAS_FALLEN when lying");
        check(!controller.getting_up(), "getting_up() is always false (recovery is a NUbots_K1 policy)");

        mj_deleteData(d);
        mj_deleteModel(m);
    }

    // ---------------------------------------------------------------------
    // Test 5: WALKING/SOCCER mode requests are wire-compatible but map to
    // PREPARE (the walk policy lives in NUbots_K1 now).
    // ---------------------------------------------------------------------
    void test_walking_maps_to_prepare() {
        std::printf("test_walking_maps_to_prepare:\n");
        mjModel* m = load_test_model();
        mjData* d  = mj_makeData(m);
        reset_to_keyframe(m, d, "ready");

        LocomotionController controller(locomotion_cfg(), gains_cfg());
        controller.request_mode_change(booster::SOCCER);

        const ModelMap map = ModelMap::build(m);
        for (int i = 0; i < 3000; ++i) {  // 3 s @ 1 kHz
            controller.step(m, d);
            mj_step(m, d);
        }

        check(controller.mode() == booster::PREPARE, "ChangeMode(SOCCER) reports PREPARE");
        const BaseState s = read_base_state(d, map);
        check(s.z > kStandHeightMin && s.z < kStandHeightMax,
              "robot holds the ready pose (z=" + std::to_string(s.z) + ")");

        mj_deleteData(d);
        mj_deleteModel(m);
    }

    void test_command_timeout_and_latency() {
        mjModel* m = load_test_model();
        mjData* d  = mj_makeData(m);
        reset_to_keyframe(m, d, "ready");
        const auto gains              = gains_cfg();
        auto target                   = load_joint_array(gains["ready_pose"]);
        target[JointIndexK1::HeadYaw] = .4;
        const auto motors      = make_low_cmd(target, load_joint_array(gains["kp"]), load_joint_array(gains["kd"]));
        auto cfg               = locomotion_cfg();
        cfg["low_cmd_timeout"] = 1e-9;
        LocomotionController stale(cfg, gains);
        stale.request_mode_change(booster::CUSTOM);
        stale.step(m, d);
        stale.set_low_cmd(1, motors);
        stale.step(m, d);
        const auto map = ModelMap::build(m);
        check(stale.command_samples() == 0, "expired command is not counted as physically applied");
        check(std::abs(d->ctrl[map.act_id[JointIndexK1::HeadYaw]]) < 1e-6,
              "timeout holds the current head pose instead of pursuing the expired target");
        LocomotionController fresh(locomotion_cfg(), gains);
        fresh.request_mode_change(booster::CUSTOM);
        fresh.step(m, d);
        fresh.set_low_cmd(1, motors);
        fresh.step(m, d);
        fresh.step(m, d);
        check(fresh.command_samples() == 1, "new command is counted once, not at every physics step");
        check(d->ctrl[map.act_id[JointIndexK1::HeadYaw]] > 0., "default one-shot command remains supported");
        mj_deleteData(d);
        mj_deleteModel(m);
    }

}  // namespace

int main() {
    test_prepare();
    test_head();
    test_custom_low_cmd();
    test_fall_detection();
    test_walking_maps_to_prepare();
    test_command_timeout_and_latency();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
