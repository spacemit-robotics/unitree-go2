#include "policy.hpp"
#include "controller.hpp"
#include "rl_service.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace go2 {
using namespace detail;  // NOLINT(build/namespaces)
namespace {
Eigen::VectorXf to_eigen(const std::vector<float>& values) {
    Eigen::VectorXf result(static_cast<Eigen::Index>(values.size()));
    for (std::size_t index = 0; index < values.size(); ++index) {
        result[static_cast<Eigen::Index>(index)] = values[index];
    }
    return result;
}

Action infer_action(rl_policy::PolicyExecutor& model, const std::vector<float>& observation) {
    if (!std::all_of(observation.begin(), observation.end(), [](float value) { return std::isfinite(value); })) {
        throw std::invalid_argument("policy observation contains NaN or infinity");
    }
    std::vector<double> result;
    model.Infer(to_eigen(observation), result);
    if (result.size() != kNumJoints) {
        throw std::runtime_error("policy action dimension mismatch");
    }
    Action action{};
    for (std::size_t index = 0; index < action.size(); ++index) {
        if (!std::isfinite(result[index])) {
            throw std::runtime_error("policy action contains NaN or infinity");
        }
        action[index] = static_cast<float>(result[index]);
    }
    return action;
}

}  // namespace

Policy::Policy(
    const std::string& model_directory, const std::string& config_path, std::shared_ptr<const PolicyAdapter> adapter) {
    if (model_directory.empty()) throw std::invalid_argument("model directory must not be empty");
    const auto yaml_path = config_path.empty()
        ? (std::filesystem::path(model_directory).parent_path() / "config/policies.yaml").string()
        : config_path;
    adapter_ = adapter ? std::move(adapter) : std::make_shared<PolicyAdapter>(yaml_path, model_directory);
    const auto config = YAML::LoadFile(yaml_path);
    const auto modes = config["go2_modes"];
    if (!modes.IsMap()) throw std::invalid_argument("missing go2_modes configuration");
    std::lock_guard<std::mutex> execution_lock(adapter_->execution_mutex_);
    std::set<Mode> configured;
    for (const auto& entry : modes) {
        const auto mode = parse_mode(entry.first.as<std::string>());
        (void)adapter_->spec_for(mode);
        if (!configured.insert(mode).second) throw std::invalid_argument("duplicate Go2 policy mode");
        const auto names = entry.second["models"].as<std::vector<std::string>>();
        const std::size_t expected_variants = 1;
        if (names.size() != expected_variants)
            throw std::invalid_argument("invalid model variant count: " + entry.first.as<std::string>());
        try {
            models_.emplace(mode, &adapter_->executor(mode));
            if (const auto motion = entry.second["motion"]) {
                auto reference = load_motion_reference(
                    (std::filesystem::path(yaml_path).parent_path() / motion["path"].as<std::string>()).string(),
                    motion["frames"].as<unsigned>());
                initial_reference_quaternions_[mode] = reference.front().quaternion;
                motion_references_[mode] = std::move(reference);
                yaw_alignments_[mode] = {1, 0, 0, 0};
            }
        } catch (const std::exception& e) {
            models_.erase(mode);
            if (entry.second["required"] && entry.second["required"].as<bool>()) throw;
            load_errors_[mode] = e.what();
        }
    }
    for (const auto mode : {Mode::kWalk, Mode::kRun, Mode::kTerrain, Mode::kWave, Mode::kRightFlip, Mode::kForwardJump,
                Mode::kSit}) {
        if (!configured.count(mode))
            throw std::invalid_argument(std::string("missing policy mode: ") + mode_name(mode));
    }
    // The controller can enter walk automatically after a motion finishes.
    for (const auto mode : {Mode::kWalk, Mode::kRun, Mode::kTerrain, Mode::kWave, Mode::kRightFlip, Mode::kForwardJump,
                Mode::kSit})
        if (!has_model(mode))
            throw std::invalid_argument(
                std::string("required policy unavailable: ") + mode_name(mode) + ": " + load_error(mode));
    if (!motion_references_.count(Mode::kForwardJump))
        throw std::invalid_argument("missing motion reference configuration");
    const std::vector<float> zeros(69U, 0.0F);
    auto& right_flip = *models_.at(Mode::kRightFlip);
    float time_step = 0.0F;
    right_flip.SetModelInput("time_step", &time_step, 1);
    (void)infer_action(right_flip, zeros);
    const auto& body_quaternion = right_flip.GetModelOutput("body_quat_w");
    if (body_quaternion.size() != 52U) throw std::runtime_error("right-flip body quaternion output mismatch");
    initial_reference_quaternions_[Mode::kRightFlip] =
        normalize({body_quaternion[0], body_quaternion[1], body_quaternion[2], body_quaternion[3]});
    yaw_alignments_[Mode::kRightFlip] = {1, 0, 0, 0};
}

Policy::~Policy() = default;

Action Policy::infer(Mode mode, const std::vector<float>& input_observation,
    const RobotState* state, std::uint64_t policy_frame) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_model(mode)) throw std::logic_error("policy unavailable: " + load_error(mode));
    auto& model = *models_.at(mode);
    if (input_observation.size() != static_cast<std::size_t>(model.ObsDim()))
        throw std::invalid_argument("policy observation dimension mismatch");
    std::vector<float> observation = input_observation;
    float time_step = 0.0F;
    if (mode == Mode::kForwardJump) {
        if (state == nullptr) throw std::invalid_argument("motion policy requires robot state");
        const auto& reference = motion_references_.at(mode);
        const auto frame = policy_frame == std::numeric_limits<std::uint64_t>::max() ? steps_[mode]++ : policy_frame;
        if (frame == 0U)
            yaw_alignments_[mode] =
                motion_yaw_alignment(state->imu.six_axis_quaternion, initial_reference_quaternions_.at(mode));
        const auto index = std::min<std::size_t>(frame, reference.size() - 1);
        const auto& ref = reference[index];
        Action previous{};
        std::copy_n(
            input_observation.begin() + adapter_->observation_term_offset(mode, "last_action"), 12, previous.begin());
        observation = adapter_->build_motion_observation(mode, *state, ref.position,
            index + 1 == reference.size() ? Joints{} : ref.velocity, ref.quaternion, previous,
            yaw_alignments_.at(mode));
    }
    if (mode == Mode::kRightFlip) {
        const bool automatic_frame = policy_frame == std::numeric_limits<std::uint64_t>::max();
        std::uint64_t frame = policy_frame;
        if (automatic_frame) frame = steps_[mode]++;
        time_step = static_cast<float>(frame);
        if (state != nullptr) {
            if (frame == 0U) {
                yaw_alignments_[mode] =
                    motion_yaw_alignment(state->imu.six_axis_quaternion, initial_reference_quaternions_.at(mode));
            }
            Joints position{}, velocity{};
            Quaternion reference{};
            {
                std::lock_guard<std::mutex> execution_lock(adapter_->execution_mutex_);
                const std::vector<float> zeros(69U, 0.0F);
                model.SetModelInput("time_step", &time_step, 1);
                (void)infer_action(model, zeros);
                const auto& reference_position = model.GetModelOutput("joint_pos");
                const auto& reference_velocity = model.GetModelOutput("joint_vel");
                const auto& body_quaternion = model.GetModelOutput("body_quat_w");
                if (reference_position.size() != kNumJoints || reference_velocity.size() != kNumJoints ||
                    body_quaternion.size() != 52U)
                    throw std::runtime_error("motion reference output dimension mismatch");
                std::copy(reference_position.begin(), reference_position.end(), position.begin());
                std::copy(reference_velocity.begin(), reference_velocity.end(), velocity.begin());
                reference = {body_quaternion[0], body_quaternion[1], body_quaternion[2], body_quaternion[3]};
            }
            Action previous{};
            std::copy_n(input_observation.begin() + adapter_->observation_term_offset(mode, "last_action"), kNumJoints,
                previous.begin());
            observation = adapter_->build_motion_observation(
                mode, *state, position, velocity, reference, previous, yaw_alignments_[mode]);
        }
    }

    std::lock_guard<std::mutex> execution_lock(adapter_->execution_mutex_);
    if (mode == Mode::kRightFlip) {
        model.SetModelInput("time_step", &time_step, 1);
    }
    return infer_action(model, observation);
}

void Policy::reset_mode(Mode mode, const RobotState* state) {
    std::lock_guard<std::mutex> lock(mutex_);
    steps_[mode] = 0U;
    if (mode != Mode::kRightFlip && mode != Mode::kForwardJump) return;

    const Quaternion actual = state == nullptr ? Quaternion{1, 0, 0, 0} : normalize(state->imu.six_axis_quaternion);
    yaw_alignments_[mode] = motion_yaw_alignment(actual, initial_reference_quaternions_.at(mode));
}

bool Policy::has_model(Mode mode) const { return models_.count(mode) != 0; }

const std::string& Policy::load_error(Mode mode) const {
    static const std::string empty;
    const auto it = load_errors_.find(mode);
    return it != load_errors_.end() ? it->second : empty;
}

}  // namespace go2
