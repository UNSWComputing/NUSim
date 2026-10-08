// Acceptance on a real six-robot scene: controllers run before one shared step,
// prefixed sensors remain distinct, and per-robot reset does not clear a neighbour.
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>

#include "module/Locomotion/src/LocomotionController.hpp"
#include "module/Simulation/src/SimCore.hpp"
#include "shared/sim/MatchConfig.hpp"
#include "shared/util/Config.hpp"

namespace {

    void require(bool condition, const char* label) {
        if (!condition) {
            throw std::runtime_error(label);
        }
    }

    class RecordingController : public k1sim::StepController {
    public:
        k1sim::module::LocomotionController servo;
        std::vector<double> times;
        RecordingController(int index)
            : servo(k1sim::config::load("locomotion.yaml"),
                    k1sim::config::load("gains.yaml"),
                    k1sim::robot_prefix(index)) {}
        void step(const mjModel* m, mjData* d) override {
            times.push_back(d->time);
            servo.step(m, d);
        }
        void reset() override {
            servo.reset();
        }
        int mode() const override {
            return servo.mode();
        }
        int fall_state() const override {
            return servo.fall_state();
        }
        bool getting_up() const override {
            return servo.getting_up();
        }
    };

}  // namespace

int main() {
    try {
        using namespace k1sim;
        const auto gains = config::load("gains.yaml");
        const auto game  = config::game(config::load("simulation.yaml"), 3, true);
        SimCore::Config cfg;
        cfg.model_path = config::field_scene(config::load("simulation.yaml"), game.field);
        cfg.robots     = 6;
        cfg.spawns     = game.spawns;
        cfg.identities = load_match_config(config::load("match_3v3.yaml")).robots;
        for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
            cfg.kp[j] = gains["kp"][j].as<double>();
            cfg.kd[j] = gains["kd"][j].as<double>();
        }
        SimCore sim(cfg);
        sim.load_model();
        std::vector<std::shared_ptr<RecordingController>> controllers;
        for (int i = 0; i < 6; ++i) {
            controllers.push_back(std::make_shared<RecordingController>(i));
            sim.set_robot_controller(i + 1, controllers.back());
            const auto& map = sim.model_map(i + 1);
            require(map.root_body_id == mj_name2id(sim.model(), mjOBJ_BODY, cfg.identities[i].body().c_str()),
                    "robot map selected another robot's root");
            require(map.sens_quat >= 0 && map.sens_gyro >= 0 && map.sens_acc >= 0,
                    "attached robot lost its real IMU sensors");
            if (i > 0) {
                const auto& previous = sim.model_map(i);
                require(map.root_qpos_adr != previous.root_qpos_adr && map.act_id[HeadYaw] != previous.act_id[HeadYaw]
                            && map.sens_gyro != previous.sens_gyro && map.sens_acc != previous.sens_acc,
                        "robot sensor/actuator maps overlap");
            }
            controllers.back()->servo.request_mode_change(booster::CUSTOM);
        }
        sim.step_once();  // Enter CUSTOM before sending the new session's joint commands.
        for (int i = 0; i < 6; ++i) {
            std::vector<message::MotorCmdData> motors(JOINT_COUNT);
            for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                motors[j].q  = gains["ready_pose"][j].as<float>();
                motors[j].kp = gains["kp"][j].as<float>();
                motors[j].kd = gains["kd"][j].as<float>();
            }
            motors[HeadYaw].q = i % 2 == 0 ? 0.4f : -0.4f;
            controllers[i]->servo.set_low_cmd(1, motors);
        }
        sim.step_once();
        require(sim.data()->ctrl[sim.model_map(1).act_id[HeadYaw]] > 0.0
                    && sim.data()->ctrl[sim.model_map(2).act_id[HeadYaw]] < 0.0,
                "different commands did not reach different actuators");
        for (int step = 0; step < 300; ++step) {
            sim.step_once();
        }
        auto before = sim.capture_states();
        require(before.robots.size() == 6, "snapshot omitted a robot");
        for (int i = 0; i < 6; ++i) {
            const auto& state = before.robots[i];
            require(state.identity.robot_id == i + 1 && state.identity.model_index == i,
                    "sensor identity does not match scene order");
            require(state.head.valid, "attached robot has no head pose");
            require(state.sim_time == before.robots[0].sim_time && state.step_count == before.robots[0].step_count,
                    "robot sensors were captured at different physics steps");
            require(controllers[i]->times == controllers[0]->times,
                    "a controller ran after another robot advanced the world");
            require(state.mode == booster::CUSTOM, "one robot did not get its controller");
        }
        require(before.robots[0].joints[HeadYaw].q > 0.2 && before.robots[1].joints[HeadYaw].q < -0.2,
                "independent joint targets are not visible in sensor streams");
        require(before.robots[0].head.quat != before.robots[1].head.quat, "head poses were copied from the main robot");
        require(before.robots[0].imu.quat != before.robots[3].imu.quat,
                "opposing team's IMU orientation was copied from the main robot");

        // Only robot 1 changes mode; the other five remain in CUSTOM.
        controllers[0]->servo.request_mode_change(booster::DAMPING);
        sim.step_once();
        before = sim.capture_states();
        require(before.robots[0].mode == booster::DAMPING && before.robots[1].mode == booster::CUSTOM,
                "mode change crossed robot boundaries");
        const auto time  = sim.data()->time;
        const auto steps = sim.step_count();
        const int ball   = mj_name2id(sim.model(), mjOBJ_JOINT, "ball_free");
        std::vector<double> ball_pose;
        if (ball >= 0) {
            ball_pose.assign(sim.data()->qpos + sim.model()->jnt_qposadr[ball],
                             sim.data()->qpos + sim.model()->jnt_qposadr[ball] + 7);
        }
        sim.reset_robot(1);
        const auto after = sim.capture_states();
        require(sim.data()->time == time && sim.step_count() == steps, "robot reset changed the world clock");
        require(after.robots[0].reset_count == 1 && after.robots[0].mode == booster::PREPARE,
                "reset did not clear the selected controller");
        require(std::abs(after.robots[0].joints[HeadYaw].q) < 1e-12, "reset did not restore selected robot pose");
        for (int i = 1; i < 6; ++i) {
            require(after.robots[i].mode == before.robots[i].mode
                        && after.robots[i].fall_state == before.robots[i].fall_state && after.robots[i].reset_count == 0
                        && after.robots[i].identity.robot_id == before.robots[i].identity.robot_id,
                    "reset changed another robot's controller or identity");
            for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
                require(after.robots[i].joints[j].q == before.robots[i].joints[j].q
                            && after.robots[i].joints[j].dq == before.robots[i].joints[j].dq,
                        "reset changed another robot's joint state");
            }
        }
        if (ball >= 0) {
            for (std::size_t i = 0; i < ball_pose.size(); ++i) {
                require(ball_pose[i] == sim.data()->qpos[sim.model()->jnt_qposadr[ball] + i],
                        "robot reset moved the ball");
            }
        }
        sim.step_once();
        require(controllers[1]->mode() == booster::CUSTOM && sim.data()->ctrl[sim.model_map(2).act_id[HeadYaw]] != 0.0,
                "neighbour's command buffer was cleared by robot reset");

        // Force only robot 2 into a fallen pose; its IMU/fall state must be distinct.
        const auto& fallen   = sim.model_map(2);
        const mjtNum axis[3] = {1, 0, 0};
        mju_axisAngle2Quat(sim.data()->qpos + fallen.root_qpos_adr + 3, axis, 1.5);
        mj_forward(sim.model(), sim.data());
        sim.step_once();
        const auto fall = sim.capture_states();
        require(fall.robots[1].fall_state == booster::HAS_FALLEN && fall.robots[0].fall_state == booster::IS_READY,
                "fall detection used another robot's pose");
        require(fall.robots[1].imu.quat != fall.robots[0].imu.quat, "fallen robot's IMU is not independent");
        sim.reset();
        const auto reset = sim.capture_states();
        require(sim.data()->time == 0.0 && sim.step_count() == 0, "full reset did not reset physics time");
        for (const auto& state : reset.robots) {
            require(state.mode == booster::PREPARE && state.fall_state == booster::IS_READY,
                    "full reset retained stale controller state");
        }
        std::puts("six-robot control, sensor and reset isolation passed");
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
    return 0;
}
