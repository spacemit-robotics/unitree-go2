/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef POLICY_ADAPTER_HPP
#define POLICY_ADAPTER_HPP
#include "controller.hpp"
#include "rl_service.h"
#include <map>
#include <memory>
#include <mutex>
#include <limits>
#include <utility>
#include <vector>

namespace go2 {
// Parse Go2 YAML defaults; adapters may inspect optional models before loading.
rl_policy::LoadedPolicyConfig LoadPolicyConfigFromYaml(const std::string& yaml_path, const std::string& policy_name,
    const std::string& model_directory, bool require_model_file = true);

using Quaternion = std::array<float, 4>;

constexpr float kSkillControlDt = 0.02F;
constexpr std::uint64_t kForwardJumpFrames = 120U;

constexpr float kSitEndSeconds = 2.0F;
constexpr float kSitRecoverySeconds = 3.4F;
constexpr float kSitRecoveryTimeoutSeconds = 6.0F;

struct SitReference {
    Joints position;
    float seated;
    float rising;
};
SitReference sit_reference(
    const std::vector<Joints>& trajectory, float time, float stand_start = std::numeric_limits<float>::infinity());

struct WaveReference {
    Joints position;
    float lift;
    float phase;
};
unsigned next_wave_tick(unsigned tick, bool raised);
WaveReference wave_reference(float time);

struct MotionReference {
    Joints position, velocity;
    Quaternion quaternion;
};
std::vector<MotionReference> load_motion_reference(const std::string& path, unsigned expected_frames);
std::vector<Joints> load_sit_reference(const std::string& path);

Quaternion motion_yaw_alignment(const Quaternion& actual, const Quaternion& reference);

// 每模式的关节侧 PD 增益表。
struct GainSet {
    Joints kp{};
    Joints kd{};
};

struct ModelSpec {
    rl_policy::LoadedPolicyConfig loaded;
    Joints default_pos{}, last_action_scale{}, lower{}, upper{}, raised_scale{}, effort_limit{};
    std::array<std::size_t, kNumJoints> mapping{};
    bool board_quaternion = false;
    float target_clip = 0, last_action_clip = 0;
    float velocity_limit = 0;
    float vx_lo{}, vx_hi{}, vy_lo{}, vy_hi{}, yaw_lo{}, yaw_hi{};
};

// Per-node configuration and executors shared by observation assembly and inference.
class PolicyAdapter {
public:
    PolicyAdapter(const std::string& yaml_path, const std::string& model_directory);
    std::vector<float> build_observation(const RobotState&, VelocityCommand, const Action& previous, Mode mode,
        float reference_time = 0.0F, const std::vector<Joints>& sit_trajectory = {}) const;
    std::vector<float> build_sit_observation(const std::vector<Joints>& trajectory, const RobotState&, const Action&,
        float time, float stand_start = std::numeric_limits<float>::infinity()) const;
    Joints sit_hardware_targets(const std::vector<Joints>& trajectory, const Action&, float time,
        float stand_start = std::numeric_limits<float>::infinity()) const;
    std::vector<float> build_motion_observation(Mode mode, const RobotState&, const Joints& reference_position,
        const Joints& reference_velocity, const Quaternion& reference_quaternion, const Action& previous,
        const Quaternion& right_yaw_alignment = {1, 0, 0, 0}) const;
    Joints action_to_hardware_targets(const Action&, Mode mode, float reference_time = 0.0F,
        const std::vector<Joints>& sit_trajectory = {}, std::size_t variant_index = 0) const;
    GainSet policy_gains(Mode mode, std::size_t variant_index = 0) const;
    const std::array<std::size_t, kNumJoints>& policy_to_hardware(Mode mode) const;
    const ModelSpec& spec_for(Mode mode) const;
    Joints policy_torques(const Joints& target, const RobotState& state, Mode mode) const;
    const ModelSpec& variant(Mode mode, std::size_t index = 0) const;
    std::vector<float> build_variant_observation(
        const RobotState&, VelocityCommand, const Action&, Mode, std::size_t variant_index) const;
    std::size_t observation_term_offset(Mode, const std::string&) const;
    std::vector<int> observation_term_dimensions(Mode mode) const;

private:
    friend class Policy;
    // Call only while holding execution_mutex_; each mode loads one session.
    rl_policy::PolicyExecutor& executor(Mode) const;
    VelocityCommand command_for_mode(VelocityCommand, Mode, std::size_t index = 0) const;
    const Joints& default_joint_position(Mode mode) const;
    std::vector<float> observation_frame(Mode, const RobotState&, VelocityCommand, const Action&, const ModelSpec&,
        const std::unordered_map<std::string, std::vector<float>>& custom = {}) const;
    std::map<Mode, std::vector<ModelSpec>> models_;
    mutable std::mutex execution_mutex_;
    mutable std::map<Mode, std::unique_ptr<rl_policy::PolicyExecutor>> executors_;
};

}  // namespace go2

namespace go2::detail {
Quaternion normalize(Quaternion quaternion);
}

#endif  // POLICY_ADAPTER_HPP
