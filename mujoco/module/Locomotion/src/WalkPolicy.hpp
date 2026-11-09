#ifndef K1SIM_MODULE_LOCOMOTION_WALKPOLICY_HPP
#define K1SIM_MODULE_LOCOMOTION_WALKPOLICY_HPP

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "shared/k1/JointIndex.hpp"

namespace k1sim::module {

    // A velocity-tracking walk policy in the export format of booster_mjlab's web demo: a directory holding
    // scene.json (joint defaults, action scales, PD gains, control period, command limits, layer list) and
    // policy.bin (the little-endian float32 weights the layers refer to).
    //
    // The network maps one observation - base angular velocity (3), gravity in the base frame (3), joint positions
    // relative to the default pose (22), joint velocities (22), the previous action (22) and the commanded
    // velocity vx, vy, wz (3) - to one action per joint; the joint target is default + action * scale.
    struct WalkPolicy {
        static constexpr std::size_t OBSERVATION_SIZE = 3 + 3 + 3 * JOINT_COUNT + 3;

        std::array<double, JOINT_COUNT> default_pose{};
        std::array<double, JOINT_COUNT> action_scale{};
        std::array<double, JOINT_COUNT> kp{};
        std::array<double, JOINT_COUNT> kd{};
        double control_dt = 0.02;                              // seconds between two network evaluations
        std::array<std::array<double, 2>, 3> command_limits{}; // vx, vy, wz: {min, max}

        // Loads <dir>/scene.json and <dir>/policy.bin. Throws std::runtime_error with a readable reason.
        static WalkPolicy load(const std::filesystem::path& dir) {
            std::ifstream scene_file(dir / "scene.json");
            if (!scene_file.is_open()) {
                throw std::runtime_error("no scene.json in " + dir.string());
            }
            const nlohmann::json scene = nlohmann::json::parse(scene_file);

            WalkPolicy policy;
            const auto& joints = scene.at("joints");
            if (joints.size() != JOINT_COUNT) {
                throw std::runtime_error("the policy drives " + std::to_string(joints.size()) + " joints, the K1 has 22");
            }
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                policy.default_pose[i] = joints[i].at("default").get<double>();
                policy.action_scale[i] = joints[i].at("action_scale").get<double>();
                policy.kp[i]           = joints[i].at("kp").get<double>();
                policy.kd[i]           = joints[i].at("kd").get<double>();
            }
            policy.control_dt = scene.value("control_dt", 0.02);
            const char* axes[3] = {"vx", "vy", "wz"};
            for (int i = 0; i < 3; ++i) {
                const auto& limit        = scene.at("command_limits").at(axes[i]);
                policy.command_limits[i] = {limit[0].get<double>(), limit[1].get<double>()};
            }

            const auto& spec = scene.at("policy");
            if (spec.at("input_dim").get<std::size_t>() != OBSERVATION_SIZE || spec.at("output_dim").get<std::size_t>() != JOINT_COUNT) {
                throw std::runtime_error("the policy's input/output size does not match the K1 observation layout");
            }
            std::ifstream weights(dir / spec.at("file").get<std::string>(), std::ios::binary | std::ios::ate);
            if (!weights.is_open()) {
                throw std::runtime_error("no " + spec.at("file").get<std::string>() + " in " + dir.string());
            }
            policy.weights_.resize(static_cast<std::size_t>(weights.tellg()) / sizeof(float));
            weights.seekg(0);
            weights.read(reinterpret_cast<char*>(policy.weights_.data()),
                         static_cast<std::streamsize>(policy.weights_.size() * sizeof(float)));

            for (const auto& layer : spec.at("layers")) {
                Layer l;
                const std::string op = layer.at("op").get<std::string>();
                if (op == "sub" || op == "div") {
                    l.kind   = op == "sub" ? Layer::Kind::SUB : Layer::Kind::DIV;
                    l.offset = layer.at("offset").get<std::size_t>();
                    l.in = l.out = layer.at("size").get<std::size_t>();
                    policy.require(l.offset + l.in);
                }
                else if (op == "gemm") {
                    l.kind   = Layer::Kind::LINEAR;
                    l.in     = layer.at("in").get<std::size_t>();
                    l.out    = layer.at("out").get<std::size_t>();
                    l.offset = layer.at("w_offset").get<std::size_t>();
                    l.bias   = layer.at("b_offset").get<std::size_t>();
                    policy.require(l.offset + l.in * l.out);
                    policy.require(l.bias + l.out);
                }
                else if (op == "elu") {
                    l.kind  = Layer::Kind::ELU;
                    l.alpha = layer.value("alpha", 1.0F);
                }
                else if (op == "relu" || op == "tanh") {
                    l.kind = op == "relu" ? Layer::Kind::RELU : Layer::Kind::TANH;
                }
                else {
                    throw std::runtime_error("unsupported policy layer '" + op + "'");
                }
                policy.layers_.push_back(l);
            }
            return policy;
        }

        std::array<double, JOINT_COUNT> evaluate(const std::array<float, OBSERVATION_SIZE>& observation) const {
            std::vector<float> x(observation.begin(), observation.end());
            std::vector<float> y;
            for (const Layer& l : layers_) {
                switch (l.kind) {
                    case Layer::Kind::SUB:
                        for (std::size_t i = 0; i < x.size(); ++i) x[i] -= weights_[l.offset + i];
                        break;
                    case Layer::Kind::DIV:
                        for (std::size_t i = 0; i < x.size(); ++i) x[i] /= weights_[l.offset + i];
                        break;
                    case Layer::Kind::LINEAR:
                        y.assign(l.out, 0.0F);
                        for (std::size_t r = 0; r < l.out; ++r) {
                            const float* row = &weights_[l.offset + r * l.in];
                            float sum        = weights_[l.bias + r];
                            for (std::size_t c = 0; c < l.in; ++c) sum += row[c] * x[c];
                            y[r] = sum;
                        }
                        x.swap(y);
                        break;
                    case Layer::Kind::ELU:
                        for (float& v : x) v = v > 0.0F ? v : l.alpha * (std::exp(v) - 1.0F);
                        break;
                    case Layer::Kind::RELU:
                        for (float& v : x) v = v > 0.0F ? v : 0.0F;
                        break;
                    case Layer::Kind::TANH:
                        for (float& v : x) v = std::tanh(v);
                        break;
                }
            }
            std::array<double, JOINT_COUNT> action{};
            for (std::size_t i = 0; i < JOINT_COUNT && i < x.size(); ++i) {
                action[i] = x[i];
            }
            return action;
        }

    private:
        struct Layer {
            enum class Kind { SUB, DIV, LINEAR, ELU, RELU, TANH } kind = Kind::ELU;
            std::size_t in = 0, out = 0, offset = 0, bias = 0;
            float alpha = 1.0F;
        };

        void require(std::size_t count) const {
            if (count > weights_.size()) {
                throw std::runtime_error("policy.bin is shorter than scene.json says");
            }
        }

        std::vector<float> weights_;
        std::vector<Layer> layers_;
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_LOCOMOTION_WALKPOLICY_HPP
