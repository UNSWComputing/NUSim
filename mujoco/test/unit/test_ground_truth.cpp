#include <cmath>
#include <stdexcept>

#include "module/Simulation/src/SimCore.hpp"
#include "shared/util/Config.hpp"
namespace {
    void require(bool value, const char* message) {
        if (!value)
            throw std::runtime_error(message);
    }
}  // namespace
int main() {
    using namespace k1sim;
    SimCore::Config cfg;
    cfg.model_path   = config::field_scene(config::load("simulation.yaml"), "kidsize");
    cfg.robots       = 2;
    const auto gains = config::load("gains.yaml");
    for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
        cfg.kp[j] = gains["kp"][j].as<double>();
        cfg.kd[j] = gains["kd"][j].as<double>();
    }
    SimCore sim(cfg);
    sim.load_model();
    const auto first = sim.capture_states();
    require(first.ball_valid, "ball missing");
    require(first.session_id[14] == '4', "session is not UUID v4");
    const int geom = mj_name2id(sim.model(), mjOBJ_GEOM, "ball");
    const int body = sim.model()->geom_bodyid[geom];
    require(std::abs(first.ball_centre[0] - sim.data()->xpos[3 * body]) > 1, "ball geom offset lost");
    for (int i = 0; i < 3; ++i)
        require(std::abs(first.ball_centre[i] - sim.data()->geom_xpos[3 * geom + i]) < 1e-12, "ball centre wrong");
    const int joint = sim.model()->body_jntadr[body], q = sim.model()->jnt_qposadr[joint],
              v             = sim.model()->jnt_dofadr[joint];
    sim.data()->qpos[q + 3] = std::cos(0.3);
    sim.data()->qpos[q + 6] = std::sin(0.3);
    sim.data()->qvel[v]     = 0.4;
    sim.data()->qvel[v + 5] = 0.7;
    mj_forward(sim.model(), sim.data());
    const auto moving = sim.capture_states();
    // An offset sphere centre has v_origin + omega cross offset. Verify world-space velocity.
    const double ox = sim.data()->geom_xpos[3 * geom] - sim.data()->xpos[3 * body];
    const double oy = sim.data()->geom_xpos[3 * geom + 1] - sim.data()->xpos[3 * body + 1];
    require(std::abs(moving.ball_velocity[0] - (0.4 - 0.7 * oy)) < 1e-10, "ball x velocity lost rotation offset");
    require(std::abs(moving.ball_velocity[1] - 0.7 * ox) < 1e-10, "ball y velocity lost rotation offset");
    sim.step_once();
    const auto before = sim.capture_states();
    sim.reset_robot(1);
    const auto reset = sim.capture_states();
    require(reset.reset_generation == before.reset_generation + 1, "per-robot reset did not invalidate world");
    require(reset.sample_sequence > before.sample_sequence && reset.session_id == before.session_id,
            "reset changed session or reversed sequence");
    require(reset.robots[1].step_count == before.robots[1].step_count, "per-robot reset changed shared clock");
    try {
        sim.reset_robot(99);
        throw std::runtime_error("invalid reset accepted");
    }
    catch (const std::out_of_range&) {
    }
    require(sim.capture_states().reset_generation == reset.reset_generation, "rejected reset changed generation");
    sim.reset();
    const auto full = sim.capture_states();
    require(full.reset_generation == reset.reset_generation + 1 && full.robots[0].step_count == 0,
            "full reset stamp wrong");
    require(full.capture_time_unix_ns > 0, "capture timestamp missing");
}
