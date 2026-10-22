#include "policy_adapter.hpp"
#include "controller.hpp"
#include <filesystem>  // NOLINT(build/c++17)

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <set>
#include <numeric>

namespace go2::detail {
Quaternion normalize(Quaternion quaternion) {
    float norm_squared = 0.0F;
    for (float value : quaternion) norm_squared += value * value;
    if (!std::isfinite(norm_squared) || norm_squared < 1.0e-6F) {
        throw std::runtime_error("invalid policy quaternion");
    }
    const float inverse_norm = 1.0F / std::sqrt(norm_squared);
    for (float& value : quaternion) value *= inverse_norm;
    return quaternion;
}

inline Quaternion conjugate(const Quaternion& quaternion) {
    return {quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]};
}

inline Quaternion multiply(const Quaternion& lhs, const Quaternion& rhs) {
    return normalize({lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3],
        lhs[0] * rhs[1] + lhs[1] * rhs[0] + lhs[2] * rhs[3] - lhs[3] * rhs[2],
        lhs[0] * rhs[2] - lhs[1] * rhs[3] + lhs[2] * rhs[0] + lhs[3] * rhs[1],
        lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0]});
}

inline Quaternion yaw_quaternion(const Quaternion& quaternion) {
    const auto q = normalize(quaternion);
    const float yaw = std::atan2(2.0F * (q[0] * q[3] + q[1] * q[2]), 1.0F - 2.0F * (q[2] * q[2] + q[3] * q[3]));
    return {std::cos(0.5F * yaw), 0.0F, 0.0F, std::sin(0.5F * yaw)};
}

inline std::array<float, 6> right_flip_rotation_six(const Quaternion& quaternion) {
    const auto [w, x, y, z] = normalize(quaternion);
    const float r00 = 1.0F - 2.0F * (y * y + z * z);
    const float r01 = 2.0F * (x * y - z * w);
    const float r10 = 2.0F * (x * y + z * w);
    const float r11 = 1.0F - 2.0F * (x * x + z * z);
    const float r02 = 2.0F * (x * z + y * w);
    const float r12 = 2.0F * (y * z - x * w);
    return {r00, r10, r01, r11, r02, r12};
}

inline std::array<float, 3> projected_gravity(const Quaternion& quaternion) {
    const auto [w, x, y, z] = normalize(quaternion);
    return {2.0F * (w * y - x * z), -2.0F * (y * z + w * x), -(1.0F - 2.0F * (x * x + y * y))};
}

}  // namespace go2::detail

namespace go2 {

using namespace detail;  // NOLINT(build/namespaces)

namespace {

namespace fs = std::filesystem;

// yaml-cpp exposes merge keys but does not apply their defaults automatically.
YAML::Node ResolvePolicyDefaults(const YAML::Node& node, unsigned depth = 0) {
    if (!node.IsMap() || depth > 16)
        throw std::runtime_error("[PolicyConfigLoader] invalid or recursive policy defaults");
    YAML::Node result(YAML::NodeType::Map);
    if (node["<<"]) result = ResolvePolicyDefaults(node["<<"], depth + 1);
    for (const auto& entry : node) {
        const auto key = entry.first.as<std::string>();
        if (key != "<<") result[key] = YAML::Clone(entry.second);
    }
    return result;
}

template <typename T>
T NodeAs(const YAML::Node& node, const T& fallback) {
    return node ? node.as<T>() : fallback;
}

rl_policy::ModelInputSource ParseInputSource(const YAML::Node& node) {
    if (!node || !node.IsScalar()) {
        throw std::runtime_error("[PolicyConfigLoader] model_io input 缺少 source");
    }
    const std::string value = node.as<std::string>();
    if (value == "observation") return rl_policy::ModelInputSource::OBSERVATION;
    if (value == "observation_history") return rl_policy::ModelInputSource::OBSERVATION_HISTORY;
    if (value == "feedback") return rl_policy::ModelInputSource::FEEDBACK;
    if (value == "constant") return rl_policy::ModelInputSource::CONSTANT;
    if (value == "external") return rl_policy::ModelInputSource::EXTERNAL;
    throw std::runtime_error("[PolicyConfigLoader] 未知 model_io input source: " + value);
}

rl_policy::ModelOutputTarget ParseOutputTarget(const YAML::Node& node) {
    if (!node || !node.IsScalar()) {
        throw std::runtime_error("[PolicyConfigLoader] model_io output 缺少 target");
    }
    const std::string value = node.as<std::string>();
    if (value == "action") return rl_policy::ModelOutputTarget::ACTION;
    if (value == "expose") return rl_policy::ModelOutputTarget::EXPOSE;
    if (value == "ignore") return rl_policy::ModelOutputTarget::IGNORE;
    throw std::runtime_error("[PolicyConfigLoader] 未知 model_io output target: " + value);
}

std::vector<float> ParseFloatValues(const YAML::Node& node, const std::string& field) {
    if (!node) return {};
    if (node.IsSequence()) return node.as<std::vector<float>>();
    if (node.IsScalar()) return {node.as<float>()};
    throw std::runtime_error("[PolicyConfigLoader] " + field + " 必须是数值或数值序列");
}

std::vector<double> ParseDoubleValues(const YAML::Node& node, const std::string& field) {
    if (!node) return {};
    if (node.IsSequence()) return node.as<std::vector<double>>();
    if (node.IsScalar()) return {node.as<double>()};
    throw std::runtime_error("[PolicyConfigLoader] " + field + " 必须是数值或数值序列");
}

}  // namespace

rl_policy::LoadedPolicyConfig LoadPolicyConfigFromYaml(const std::string& yaml_path, const std::string& policy_name,
    const std::string& robot_dir, bool require_model_file) {
    const fs::path yaml_abs = fs::absolute(yaml_path);
    if (!fs::exists(yaml_abs)) {
        throw std::runtime_error("[PolicyConfigLoader] 配置文件不存在: " + yaml_abs.string());
    }

    YAML::Node cfg = YAML::LoadFile(yaml_abs.string());

    if (!cfg["rl_policy"] || !cfg["rl_policy"]["type"]) {
        throw std::runtime_error("[PolicyConfigLoader] 缺少 rl_policy.type 配置");
    }
    if (cfg["rl_policy"]["type"].as<std::string>() != "onnx_infer") {
        throw std::runtime_error("[PolicyConfigLoader] 不支持的 rl_policy type");
    }

    const auto rl_onnx = cfg["rl_policy"]["onnx_infer"];
    const auto policies = rl_onnx ? rl_onnx["policies"] : YAML::Node{};
    if (!policies || !policies[policy_name]) {
        throw std::runtime_error("[PolicyConfigLoader] 策略 '" + policy_name + "' 的配置不存在");
    }

    const auto policy = ResolvePolicyDefaults(policies[policy_name]);
    if (!policy["model_path"]) {
        throw std::runtime_error("[PolicyConfigLoader] 策略 '" + policy_name + "' 缺少 model_path");
    }

    rl_policy::LoadedPolicyConfig out;
    if (NodeAs(policy["backend"], std::string("onnx")) != "onnx") {
        throw std::runtime_error("[PolicyConfigLoader] backend must be onnx");
    }
    const auto runtime = policy["runtime"];
    if (runtime) {
        out.exec_cfg.runtime.provider = NodeAs(runtime["provider"], std::string("auto"));
        out.exec_cfg.runtime.threads = NodeAs(runtime["threads"], 1);
        out.exec_cfg.runtime.affinity = NodeAs(runtime["affinity"], std::string{});
        out.exec_cfg.runtime.ort_spinning = NodeAs(runtime["ort_spinning"], true);
        out.exec_cfg.runtime.ep_dump_subgraphs = NodeAs(runtime["ep_dump_subgraphs"], false);
        out.exec_cfg.runtime.ep_profile_prefix = NodeAs(runtime["ep_profile_prefix"], std::string{});
    }
    if (out.exec_cfg.runtime.threads < 1) {
        throw std::runtime_error("[PolicyConfigLoader] runtime.threads must be positive");
    }
    out.rl_dt = NodeAs(cfg["rl_policy"]["rl_dt"], 0.02);
    if (!std::isfinite(out.rl_dt) || out.rl_dt <= 0.0) {
        throw std::runtime_error("[PolicyConfigLoader] rl_policy.rl_dt 必须是正数");
    }
    const fs::path robot_dir_path(robot_dir);
    out.exec_cfg.model_path = fs::weakly_canonical(robot_dir_path / policy["model_path"].as<std::string>()).string();
    if (require_model_file && !fs::exists(out.exec_cfg.model_path)) {
        throw std::runtime_error("[PolicyConfigLoader] ONNX 模型文件不存在: " + out.exec_cfg.model_path);
    }

    if (policy["action_scale"]) {
        if (policy["action_scale"].IsSequence()) {
            out.exec_cfg.action_scale = policy["action_scale"].as<std::vector<double>>();
        } else {
            out.exec_cfg.action_scale = {policy["action_scale"].as<double>()};
        }
    }
    if (policy["action_blend_ratio"]) {
        out.exec_cfg.action_blend_ratio = policy["action_blend_ratio"].as<double>();
    }
    if (policy["action_joint_index"] && policy["action_joint_index"].IsSequence()) {
        out.exec_cfg.action_joint_index = policy["action_joint_index"].as<std::vector<int>>();
    }
    if (policy["rl_default_pos"] && policy["rl_default_pos"].IsSequence()) {
        out.exec_cfg.rl_default_pos = policy["rl_default_pos"].as<std::vector<double>>();
    }
    if (out.exec_cfg.rl_default_pos.empty()) {
        throw std::runtime_error("[PolicyConfigLoader] 策略 '" + policy_name + "' 缺少 rl_default_pos 配置");
    }

    const auto model_io = policy["model_io"];
    if (!model_io || !model_io.IsMap()) {
        throw std::runtime_error("[PolicyConfigLoader] 策略 '" + policy_name + "' 缺少 model_io map");
    }
    const auto inputs = model_io["inputs"];
    const auto outputs = model_io["outputs"];
    if (!inputs || !inputs.IsSequence() || inputs.size() == 0 || !outputs || !outputs.IsSequence() ||
        outputs.size() == 0) {
        throw std::runtime_error("[PolicyConfigLoader] model_io 必须包含非空 inputs 和 outputs 序列");
    }

    for (const auto& node : inputs) {
        if (!node.IsMap() || !node["name"]) {
            throw std::runtime_error("[PolicyConfigLoader] model_io input 缺少 name");
        }
        rl_policy::ModelInputBindingConfig binding;
        binding.name = node["name"].as<std::string>();
        binding.source = ParseInputSource(node["source"]);
        binding.key = NodeAs(node["key"], std::string{});
        binding.feedback_output = NodeAs(node["output"], std::string{});
        binding.history_source = NodeAs(node["history_of"], std::string{});
        binding.observation_offset = NodeAs(node["offset"], -1);
        if (node["value"] && node["default"]) {
            throw std::runtime_error(
                "[PolicyConfigLoader] model_io input 不能同时配置 value 和 default: " + binding.name);
        }
        const auto initial = node["value"] ? node["value"] : node["default"];
        binding.initial_value = ParseFloatValues(initial, "model_io input value/default");
        out.exec_cfg.model_io.inputs.push_back(std::move(binding));
    }

    for (const auto& node : outputs) {
        if (!node.IsMap() || !node["name"]) {
            throw std::runtime_error("[PolicyConfigLoader] model_io output 缺少 name");
        }
        rl_policy::ModelOutputBindingConfig binding;
        binding.name = node["name"].as<std::string>();
        binding.target = ParseOutputTarget(node["target"]);
        binding.key = NodeAs(node["key"], std::string{});
        out.exec_cfg.model_io.outputs.push_back(std::move(binding));
    }

    out.exec_cfg.clip_observations = ParseDoubleValues(policy["clip_observations"], "clip_observations");
    out.exec_cfg.clip_actions = ParseDoubleValues(policy["clip_actions"], "clip_actions");

    const auto obs = policy["observation"];
    if (obs) {
        out.exec_cfg.ang_vel_scale = NodeAs(obs["ang_vel_scale"], 1.0);
        out.exec_cfg.dof_pos_scale = NodeAs(obs["dof_pos_scale"], 1.0);
        out.exec_cfg.dof_vel_scale = NodeAs(obs["dof_vel_scale"], 1.0);
        out.exec_cfg.euler_angle_scale = NodeAs(obs["euler_angle_scale"], 1.0);
        out.exec_cfg.dof_pos_subtract_default = NodeAs(obs["dof_pos_subtract_default"], true);
        out.exec_cfg.phase_period = NodeAs(obs["phase_period"], 1.0);
        out.exec_cfg.strict_obs_dim_check = NodeAs(obs["strict_dim_check"], true);

        // 读取 flat segments（segment_0_terms / segment_1_terms / ...）
        out.exec_cfg.obs_segments.clear();
        for (int seg_idx = 0;; ++seg_idx) {
            const std::string pfx = "segment_" + std::to_string(seg_idx) + "_";
            if (!obs[pfx + "terms"]) break;

            rl_policy::ObsSegmentConfig seg;
            for (const auto& t : obs[pfx + "terms"]) {
                seg.terms.push_back(t.as<std::string>());
            }
            seg.mode = NodeAs(obs[pfx + "mode"], std::string(""));
            seg.length = NodeAs(obs[pfx + "length"], 0);
            seg.order = NodeAs(obs[pfx + "order"], std::string("oldest_first"));
            seg.include_current = NodeAs(obs[pfx + "include_current"], true);
            out.exec_cfg.obs_segments.push_back(seg);
        }

        // 读取 custom scalars（parallel arrays）
        const auto scalar_keys = obs["custom_scalar_keys"];
        const auto scalar_vals = obs["custom_scalar_values"];
        if (scalar_keys && scalar_vals && scalar_keys.IsSequence() && scalar_vals.IsSequence()) {
            const auto keys = scalar_keys.as<std::vector<std::string>>();
            const auto vals = scalar_vals.as<std::vector<double>>();
            for (size_t i = 0; i < keys.size() && i < vals.size(); ++i) {
                out.exec_cfg.custom_scalar_defaults[keys[i]] = static_cast<float>(vals[i]);
            }
        }
    }

    // 读取 custom array dims（泛型 N 维 obs term 维度声明）
    // 例：
    //   custom_array_dims:
    //     motion_command: 58
    //     motion_anchor_pos_b: 3
    //     motion_anchor_ori_b: 6
    const auto array_dims = policy["custom_array_dims"];
    if (array_dims && array_dims.IsMap()) {
        for (const auto& kv : array_dims) {
            const auto name = kv.first.as<std::string>();
            const int dim = kv.second.as<int>();
            if (dim > 0) {
                out.exec_cfg.custom_array_dims[name] = dim;
            }
        }
    }

    if (out.exec_cfg.obs_segments.empty()) {
        throw std::runtime_error("[PolicyConfigLoader] 策略 '" + policy_name + "' 缺少 observation.segments 配置");
    }

    const auto command = policy["command"];
    if (command) {
        if (command["scale"] && command["scale"].IsSequence() && command["scale"].size() >= 3) {
            out.exec_cfg.command_scale[0] = command["scale"][0].as<double>();
            out.exec_cfg.command_scale[1] = command["scale"][1].as<double>();
            out.exec_cfg.command_scale[2] = command["scale"][2].as<double>();
        }
        if (command["init"] && command["init"].IsSequence() && command["init"].size() >= 3) {
            out.command_init[0] = command["init"][0].as<double>();
            out.command_init[1] = command["init"][1].as<double>();
            out.command_init[2] = command["init"][2].as<double>();
        }
    }

    if (policy["infer_decimation"]) {
        out.infer_decimation = policy["infer_decimation"].as<int>();
    }
    out.max_roll = NodeAs(policy["max_roll"], 1.0);
    out.max_pitch = NodeAs(policy["max_pitch"], 1.0);

    const auto gait = policy["gait"];
    if (gait) {
        if (gait["cycle"]) out.exec_cfg.gait_cycle = gait["cycle"].as<double>();
        out.exec_cfg.gait_left_offset = NodeAs(gait["left_offset"], 0.0);
        out.exec_cfg.gait_right_offset = NodeAs(gait["right_offset"], 0.5);
        out.exec_cfg.gait_left_ratio = NodeAs(gait["left_ratio"], 0.5);
        out.exec_cfg.gait_right_ratio = NodeAs(gait["right_ratio"], 0.5);
    }

    if (policy["motion_length"]) {
        out.exec_cfg.motion_length = policy["motion_length"].as<double>();
    }

    if (policy["kp"] && policy["kp"].IsSequence()) {
        out.kp = policy["kp"].as<std::vector<double>>();
    }
    if (policy["kd"] && policy["kd"].IsSequence()) {
        out.kd = policy["kd"].as<std::vector<double>>();
    }

    return out;
}

namespace {
struct ObservationFrame {
    std::array<double, 3> gyro{}, rpy{}, base_velocity{};
    std::array<double, 4> quaternion{1, 0, 0, 0};
    std::vector<double> joint_position, joint_velocity, previous_action;
};
void MapPolicyActionToTargetPos(
    const rl_policy::PolicyExecutorConfig& c, const std::vector<double>& action, std::vector<double>& target_pos) {
    if (c.action_scale.empty() || (c.action_scale.size() != 1 && c.action_scale.size() != action.size()))
        throw std::invalid_argument("action_scale dimension mismatch");
    const int ndof = static_cast<int>(c.rl_default_pos.size());
    target_pos.assign(ndof, 0.0);

    // 未被策略控制的关节保持 rl_default_pos
    for (int j = 0; j < ndof; ++j) {
        target_pos[j] = c.rl_default_pos[j];
    }

    // 策略动作覆盖映射关节
    const bool per_joint_scale = c.action_scale.size() > 1;
    if (!c.action_joint_index.empty()) {
        const int n_act = static_cast<int>(std::min<std::size_t>(action.size(), c.action_joint_index.size()));
        for (int i = 0; i < n_act; ++i) {
            const int j = c.action_joint_index[i];
            if (j < 0 || j >= ndof) continue;
            const double scale = per_joint_scale ? c.action_scale[i] : c.action_scale[0];
            target_pos[j] = action[i] * scale + c.rl_default_pos[j];
        }
    } else {
        const int n_act = std::min(ndof, static_cast<int>(action.size()));
        for (int i = 0; i < n_act; ++i) {
            const double scale = per_joint_scale ? c.action_scale[i] : c.action_scale[0];
            target_pos[i] = action[i] * scale + c.rl_default_pos[i];
        }
    }
}

std::vector<int> ObservationTermDimensions(const rl_policy::PolicyExecutorConfig& config) {
    std::vector<int> dimensions;
    for (const auto& term : config.obs_segments.front().terms) {
        const auto custom = config.custom_array_dims.find(term);
        if (custom != config.custom_array_dims.end())
            dimensions.push_back(custom->second);
        else if (term == "ang_vel" || term == "gravity" || term == "command")
            dimensions.push_back(3);
        else if (term == "dof_pos" || term == "dof_vel" || term == "last_action")
            dimensions.push_back(kNumJoints);
        else
            throw std::invalid_argument("unsupported Go2 observation term: " + term);
    }
    return dimensions;
}

Joints joint_values(const YAML::Node& node, const std::string& field, float fallback) {
    Joints result;
    result.fill(fallback);
    if (!node) return result;
    const auto value = node[field];
    if (!value) return result;
    if (value.IsScalar())
        result.fill(value.as<float>());
    else {
        const auto values = value.as<std::vector<float>>();
        if (values.size() != kNumJoints) throw std::invalid_argument(field + " requires 12 values");
        std::copy(values.begin(), values.end(), result.begin());
    }
    for (auto v : result)
        if (!std::isfinite(v)) throw std::invalid_argument(field + " must be finite");
    return result;
}
}  // namespace

PolicyAdapter::PolicyAdapter(const std::string& path, const std::string& directory) {
    const auto document = YAML::LoadFile(path);
    const auto policies = document["rl_policy"]["onnx_infer"]["policies"];
    const auto modes = document["go2_modes"];
    if (!modes.IsMap()) throw std::invalid_argument("missing go2_modes");
    for (const auto& entry : modes) {
        const auto mode = parse_mode(entry.first.as<std::string>());
        if (mode == Mode::kStand || mode == Mode::kPassive || mode == Mode::kCrouch || mode == Mode::kRollover)
            throw std::invalid_argument("pose mode cannot have a policy");
        if (models_.count(mode)) throw std::invalid_argument("duplicate mode configuration");
        auto& variants = models_[mode];
        for (const auto& name : entry.second["models"].as<std::vector<std::string>>()) {
            ModelSpec spec;
            spec.loaded = LoadPolicyConfigFromYaml(path, name, directory, false);
            const auto& cfg = spec.loaded.exec_cfg;
            const auto node = policies[name]["policy_adapter"];
            if (!node.IsMap()) throw std::invalid_argument(name + ": missing policy_adapter");
            if (cfg.rl_default_pos.size() != 12 || cfg.action_joint_index.size() != 12 || spec.loaded.kp.size() != 12 ||
                spec.loaded.kd.size() != 12 || (cfg.action_scale.size() != 1 && cfg.action_scale.size() != 12))
                throw std::invalid_argument(name + ": invalid joint/default/scale/PD dimensions");
            std::set<int> indices;
            for (std::size_t i = 0; i < 12; ++i) {
                const int index = cfg.action_joint_index[i];
                if (index < 0 || index >= 12 || !indices.insert(index).second)
                    throw std::invalid_argument(name + ": action_joint_index must be a permutation of 0..11");
                spec.mapping[i] = index;
                spec.default_pos[i] = cfg.rl_default_pos[index];
            }
            for (const auto* values : std::array<const std::vector<double>*, 4>{
                        &cfg.rl_default_pos, &cfg.action_scale, &spec.loaded.kp, &spec.loaded.kd})
                for (double v : *values)
                    if (!std::isfinite(v)) throw std::invalid_argument(name + ": non-finite joint parameter");
            for (const auto* values : {&spec.loaded.kp, &spec.loaded.kd})
                for (double v : *values)
                    if (v < 0) throw std::invalid_argument(name + ": negative PD gain");
            for (double v : {cfg.ang_vel_scale, cfg.dof_pos_scale, cfg.dof_vel_scale, cfg.command_scale[0],
                        cfg.command_scale[1], cfg.command_scale[2]})
                if (!std::isfinite(v)) throw std::invalid_argument(name + ": non-finite observation scale");
            const auto limits = node["command_limits"].as<std::vector<float>>();
            if (limits.size() != 6) throw std::invalid_argument(name + ": command_limits requires 6 values");
            for (unsigned i = 0; i < 6; i += 2)
                if (!std::isfinite(limits[i]) || !std::isfinite(limits[i + 1]) || limits[i] > limits[i + 1])
                    throw std::invalid_argument(name + ": invalid command limits");
            spec.vx_lo = limits[0];
            spec.vx_hi = limits[1];
            spec.vy_lo = limits[2];
            spec.vy_hi = limits[3];
            spec.yaw_lo = limits[4];
            spec.yaw_hi = limits[5];
            const auto quaternion = node["quaternion"].as<std::string>();
            if (quaternion != "board" && quaternion != "six_axis")
                throw std::invalid_argument("unknown quaternion source");
            spec.board_quaternion = quaternion == "board";
            spec.last_action_scale = joint_values(node, "last_action_scale", 1);
            spec.lower = joint_values(node, "joint_lower", -1000);
            spec.upper = joint_values(node, "joint_upper", 1000);
            spec.raised_scale = joint_values(node, "raised_action_scale", 0);
            if (mode == Mode::kWave || mode == Mode::kSit) {
                spec.effort_limit = node["effort_limit"].as<float>();
                spec.velocity_limit = node["velocity_limit"].as<float>();
                if (!std::isfinite(spec.effort_limit) || !std::isfinite(spec.velocity_limit) ||
                    spec.effort_limit <= 0 || spec.velocity_limit <= 0)
                    throw std::invalid_argument("invalid wave actuator limits");
            }
            spec.last_action_clip = node["last_action_clip"] ? node["last_action_clip"].as<float>() : 0;
            if (!std::isfinite(spec.last_action_clip) || spec.last_action_clip < 0)
                throw std::invalid_argument("invalid last_action_clip");
            spec.target_clip = node["target_clip"].as<float>();
            if (!std::isfinite(spec.target_clip) || spec.target_clip < 0)
                throw std::invalid_argument("invalid target_clip");
            for (unsigned i = 0; i < 12; ++i)
                if (spec.lower[i] > spec.upper[i]) throw std::invalid_argument("invalid joint limits");
            if (cfg.obs_segments.size() != 1) throw std::invalid_argument("Go2 expects one observation segment");
            const auto& segment = cfg.obs_segments.front();
            const bool history = mode == Mode::kRun || mode == Mode::kTerrain;
            if (history && (segment.length != 5 || segment.mode != "flat_history" ||
                    segment.order != "oldest_first" || !segment.include_current))
                throw std::invalid_argument(name + ": history layout does not match controller reset semantics");
            std::unordered_map<std::string, int> custom_dimensions;
            if (mode == Mode::kWave || mode == Mode::kSit)
                custom_dimensions = {{"reference_delta", 12}, {"reference_phase", 2}};
            else if (mode == Mode::kRightFlip || mode == Mode::kForwardJump)
                custom_dimensions = {
                    {"reference_position", 12}, {"reference_velocity", 12}, {"reference_orientation", 6}};
            if (cfg.custom_array_dims != custom_dimensions)
                throw std::invalid_argument(name + ": custom observation dimensions do not match Go2 reference terms");
            const auto dimensions = ObservationTermDimensions(cfg);
            const int frame_dim = std::accumulate(dimensions.begin(), dimensions.end(), 0);
            const int expected = mode == Mode::kWave || mode == Mode::kSit ? 56
                : mode == Mode::kRightFlip || mode == Mode::kForwardJump ? 69 : 45;
            if (frame_dim != expected) throw std::invalid_argument(name + ": unexpected observation frame dimension");
            if (!history && !segment.mode.empty()) throw std::invalid_argument(name + ": unexpected history segment");
            const std::set<std::string> builtins{"ang_vel", "gravity", "command", "dof_pos", "dof_vel", "last_action"};
            std::set<std::string> observed;
            for (const auto& term : segment.terms) {
                if (!observed.insert(term).second || (!builtins.count(term) && !cfg.custom_array_dims.count(term)))
                    throw std::invalid_argument(name + ": duplicate or unknown observation term " + term);
            }
            if (!observed.count("last_action")) throw std::invalid_argument(name + ": missing last_action");
            if (!cfg.clip_observations.empty() && cfg.clip_observations.size() != 1)
                throw std::invalid_argument(name + ": Go2 frame clipping requires a scalar");
            for (double v : cfg.clip_observations)
                if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("invalid observation clipping");
            variants.push_back(std::move(spec));
        }
        if (variants.empty()) throw std::invalid_argument("empty policy variants");
    }
}

rl_policy::PolicyExecutor& PolicyAdapter::executor(Mode mode) const {
    auto found = executors_.find(mode);
    if (found != executors_.end()) return *found->second;
    const auto& loaded = spec_for(mode).loaded;
    if (std::abs(loaded.rl_dt - .02) > 1e-9 || loaded.infer_decimation != 1)
        throw std::invalid_argument("Go2 requires rl_dt=0.02 and infer_decimation=1");
    auto config = loaded.exec_cfg;
    for (auto& segment : config.obs_segments)
        for (auto& term : segment.terms)
            if (term == "last_action") term = "go2_previous_action";
    config.custom_array_dims["go2_previous_action"] = kNumJoints;
    auto model = std::make_unique<rl_policy::PolicyExecutor>();
    model->Init(config);
    if (model->ActionDim() != static_cast<int>(kNumJoints))
        throw std::invalid_argument("Go2 policy must output 12 actions");
    if ((mode == Mode::kRightFlip || mode == Mode::kForwardJump) && model->ObsDim() != 69)
        throw std::invalid_argument("Go2 motion tracking requires 69 observation values");
    auto& result = *model;
    executors_.emplace(mode, std::move(model));
    return result;
}

std::vector<float> PolicyAdapter::build_variant_observation(
    const RobotState& state, VelocityCommand command, const Action& previous, Mode mode, std::size_t index) const {
    return observation_frame(mode, state, command_for_mode(command, mode, index), previous, variant(mode, index));
}
std::size_t PolicyAdapter::observation_term_offset(Mode mode, const std::string& key) const {
    const auto& terms = spec_for(mode).loaded.exec_cfg.obs_segments.front().terms;
    const auto dims = observation_term_dimensions(mode);
    std::size_t offset = 0;
    for (std::size_t i = 0; i < terms.size(); ++i) {
        if (terms[i] == key) return offset;
        offset += dims[i];
    }
    throw std::invalid_argument("missing observation term: " + key);
}

std::vector<int> PolicyAdapter::observation_term_dimensions(Mode mode) const {
    return ObservationTermDimensions(spec_for(mode).loaded.exec_cfg);
}

std::vector<float> PolicyAdapter::observation_frame(Mode mode, const RobotState& state, VelocityCommand command,
    const Action& previous, const ModelSpec& spec,
    const std::unordered_map<std::string, std::vector<float>>& custom) const {
    ObservationFrame frame;
    const auto q = detail::normalize(spec.board_quaternion ? state.imu.quaternion : state.imu.six_axis_quaternion);
    const auto [w, x, y, z] = q;
    frame.rpy = {std::atan2(2. * (w * x + y * z), 1. - 2. * (x * x + y * y)),
        std::asin(std::clamp(2. * (w * y - z * x), -1., 1.)), 0};
    std::copy(q.begin(), q.end(), frame.quaternion.begin());
    std::copy(state.imu.angular_velocity.begin(), state.imu.angular_velocity.end(), frame.gyro.begin());
    frame.joint_position.resize(12);
    frame.joint_velocity.resize(12);
    frame.previous_action.resize(12);
    for (unsigned i = 0; i < 12; ++i) {
        frame.joint_position[i] = state.motors.joints[i].feedback.position_rad;
        frame.joint_velocity[i] = state.motors.joints[i].feedback.velocity_rad_s;
        frame.previous_action[i] =
            (spec.last_action_clip > 0 ? std::clamp(previous[i], -spec.last_action_clip, spec.last_action_clip)
                                        : previous[i]) *
            spec.last_action_scale[i];
    }
    const auto& config = spec.loaded.exec_cfg;
    std::lock_guard<std::mutex> lock(execution_mutex_);
    auto& model = executor(mode);
    // The controller owns accepted actions, including resets and stale-result rejection.
    // Use the public custom-array input instead of the executor's last inferred action.
    const std::vector<float> previous_values(frame.previous_action.begin(), frame.previous_action.end());
    model.SetCustomArray("go2_previous_action", previous_values.data(), previous_values.size());
    for (const auto& [name, dimension] : config.custom_array_dims) {
        const auto value = custom.find(name);
        const auto values = value == custom.end() ? std::vector<float>(dimension, 0.F) : value->second;
        model.SetCustomArray(name, values.data(), values.size());
    }
    Eigen::VectorXf assembled;
    model.AssembleObs(frame.gyro, frame.rpy, command.vx, command.vy, command.yaw,
        frame.joint_position, frame.joint_velocity, frame.quaternion, frame.base_velocity,
        static_cast<float>(spec.loaded.rl_dt), assembled);

    // Keep Go2's existing live-frame history seeding and reset behavior in Controller.
    // For term-major SDK histories, take the current sample of each term.
    const auto& segment = config.obs_segments.front();
    const bool history = segment.mode == "flat_history";
    const auto dimensions = ObservationTermDimensions(config);
    std::vector<float> result;
    std::size_t offset = 0;
    for (const auto dimension : dimensions) {
        const auto current = offset + (history ? dimension * (segment.length - 1) : 0);
        if (current + dimension > static_cast<std::size_t>(assembled.size()))
            throw std::runtime_error("assembled Go2 observation dimension mismatch");
        result.insert(result.end(), assembled.data() + current, assembled.data() + current + dimension);
        offset += dimension * (history ? segment.length : 1);
    }
    for (auto& value : result) {
        if (!std::isfinite(value)) throw std::invalid_argument("non-finite observation frame");
        if (!config.clip_observations.empty()) {
            const float limit = config.clip_observations.front();
            value = std::clamp(value, -limit, limit);
        }
    }
    return result;
}

const ModelSpec& PolicyAdapter::variant(Mode mode, std::size_t index) const { return models_.at(mode).at(index); }
const ModelSpec& PolicyAdapter::spec_for(Mode mode) const { return variant(mode); }
const Joints& PolicyAdapter::default_joint_position(Mode mode) const { return spec_for(mode).default_pos; }
VelocityCommand PolicyAdapter::command_for_mode(VelocityCommand command, Mode mode, std::size_t index) const {
    if (!std::isfinite(command.vx) || !std::isfinite(command.vy) || !std::isfinite(command.yaw))
        throw std::invalid_argument("non-finite command");
    const auto& spec = variant(mode, index);
    return {std::clamp(command.vx, spec.vx_lo, spec.vx_hi), std::clamp(command.vy, spec.vy_lo, spec.vy_hi),
        std::clamp(command.yaw, spec.yaw_lo, spec.yaw_hi)};
}

VelocityCommand PolicyAdapter::limit_command(VelocityCommand command) const {
    if (!std::isfinite(command.vx) || !std::isfinite(command.vy) || !std::isfinite(command.yaw)) {
        throw std::invalid_argument("velocity command contains NaN or infinity");
    }
    return command_for_mode(command, Mode::kWalk);
}

// 策略关节序 → 硬件关节序映射。
const std::array<std::size_t, kNumJoints>& PolicyAdapter::policy_to_hardware(Mode mode) const {
    return spec_for(mode).mapping;
}

Joints PolicyAdapter::action_to_hardware_targets(const Action& action, Mode mode, float reference_time,
    const std::vector<Joints>& sit_trajectory, std::size_t variant_index) const {
    const auto& map = policy_to_hardware(mode);
    Joints target{};
    if (mode == Mode::kSit) return sit_hardware_targets(sit_trajectory, action, reference_time);
    if (mode == Mode::kWave) {
        const auto reference = wave_reference(reference_time);
        for (std::size_t i = 0; i < kNumJoints; ++i) {
            if (!std::isfinite(action[i])) {
                throw std::invalid_argument("non-finite wave action");
            }
            const auto& spec = spec_for(mode);
            const auto& scales = spec.loaded.exec_cfg.action_scale;
            const float scale =
                scales[scales.size() == 1 ? 0 : i] * (1 - reference.lift) + spec.raised_scale[i] * reference.lift;
            target[map[i]] =
                std::clamp(reference.position[i] + scale * std::clamp(action[i], -spec.target_clip, spec.target_clip),
                    spec.lower[i], spec.upper[i]);
        }
        return target;
    }
    const auto& spec = variant(mode, variant_index);
    std::vector<double> values(action.begin(), action.end()), mapped;
    for (auto& value : values) {
        if (!std::isfinite(value)) throw std::invalid_argument("non-finite action");
        if (spec.target_clip > 0) value = std::clamp(value, -double(spec.target_clip), double(spec.target_clip));
    }
    MapPolicyActionToTargetPos(spec.loaded.exec_cfg, values, mapped);
    std::copy(mapped.begin(), mapped.end(), target.begin());
    return target;
}

Joints PolicyAdapter::action_to_hardware_targets(const Action& action) const {
    return action_to_hardware_targets(action, Mode::kWalk);
}

GainSet PolicyAdapter::policy_gains(Mode mode, std::size_t variant_index) const {
    const auto& loaded = variant(mode, variant_index).loaded;
    GainSet result;
    std::copy(loaded.kp.begin(), loaded.kp.end(), result.kp.begin());
    std::copy(loaded.kd.begin(), loaded.kd.end(), result.kd.begin());
    return result;
}

}  // namespace go2

namespace go2 {
using namespace detail;  // NOLINT(build/namespaces)

namespace {
float smooth(float x) {
    x = std::clamp(x, 0.F, 1.F);
    return std::clamp(x * x * x * (10.F + x * (-15.F + 6.F * x)), 0.F, 1.F);
}
}  // namespace

std::vector<MotionReference> load_motion_reference(const std::string& path, unsigned expected_frames) {
    std::ifstream motion(path);
    std::string magic;
    unsigned frames = 0, fps = 0;
    if (!(motion >> magic >> frames >> fps) || magic != "GO2_MOTION_V1" || frames == 0 || frames != expected_frames ||
        fps != 50)
        throw std::runtime_error(std::string("invalid motion header: ") + path);
    std::vector<MotionReference> reference;
    for (unsigned frame = 0; frame < frames; ++frame) {
        MotionReference ref{};
        for (auto* values : {&ref.position, &ref.velocity}) {
            for (float& v : *values)
                if (!(motion >> v) || !std::isfinite(v)) throw std::runtime_error("invalid motion joints");
        }
        for (float& v : ref.quaternion)
            if (!(motion >> v) || !std::isfinite(v)) throw std::runtime_error("invalid motion quaternion");
        ref.quaternion = normalize(ref.quaternion);
        reference.push_back(ref);
    }
    motion >> std::ws;
    if (!motion.eof()) throw std::runtime_error("unexpected motion trailing data");
    return reference;
}

std::vector<Joints> load_sit_reference(const std::string& path) {
    std::vector<Joints> joints;
    for (const auto& frame : load_motion_reference(path, 471U))
        joints.push_back(frame.position);
    return joints;
}

// Sample the training NPZ at 50 Hz, holding frame 100 until an exit request.
SitReference sit_reference(const std::vector<Joints>& trajectory, float time, float stand_start) {
    if (trajectory.size() != 471U) throw std::invalid_argument("sit motion requires 471 frames");
    if (!std::isfinite(time) || time < 0 || std::isnan(stand_start) || stand_start < 0)
        throw std::invalid_argument("invalid sit reference time");
    stand_start = std::max(stand_start, kSitEndSeconds);
    const float rise = smooth((time - stand_start) / kSitRecoverySeconds);
    const float seated = smooth((time - .4F) / 1.6F) * (1 - rise);
    const float clock = time >= stand_start ? 6.F + std::clamp(time - stand_start, 0.F, kSitRecoverySeconds)
                                            : std::min(time, kSitEndSeconds);
    const float index = clock * 50.F;
    const auto lo = std::min(static_cast<std::size_t>(index), trajectory.size() - 1);
    const auto hi = std::min(lo + 1, trajectory.size() - 1);
    const float weight = index - static_cast<float>(lo);
    SitReference result{{}, seated, rise};
    for (unsigned i = 0; i < 12; ++i)
        result.position[i] = trajectory[lo][i] * (1 - weight) + trajectory[hi][i] * weight;
    return result;
}

std::vector<float> PolicyAdapter::build_sit_observation(const std::vector<Joints>& trajectory, const RobotState& state,
    const Action& previous, float time, float stand_start) const {
    const auto reference = sit_reference(trajectory, time, stand_start);
    const auto& spec = spec_for(Mode::kSit);
    std::vector<float> delta(12);
    for (unsigned i = 0; i < 12; ++i) delta[i] = reference.position[i] - spec.default_pos[i];
    return observation_frame(Mode::kSit, state, {}, previous, spec,
        {{"reference_delta", delta}, {"reference_phase", {reference.seated, reference.rising}}});
}

Joints PolicyAdapter::sit_hardware_targets(
    const std::vector<Joints>& trajectory, const Action& action, float time, float stand_start) const {
    const auto reference = sit_reference(trajectory, time, stand_start);
    const auto& spec = spec_for(Mode::kSit);
    const auto& scales = spec.loaded.exec_cfg.action_scale;
    Joints target{};
    for (unsigned i = 0; i < 12; ++i) {
        if (!std::isfinite(action[i])) throw std::invalid_argument("non-finite sit action");
        target[spec.mapping[i]] = std::clamp(reference.position[i] +
                float(scales[scales.size() == 1 ? 0 : i]) * std::clamp(action[i], -spec.target_clip, spec.target_clip),
            spec.lower[i], spec.upper[i]);
    }
    return target;
}

unsigned next_wave_tick(unsigned tick, bool raised) {
    const unsigned start = tick == 0U && raised ? 30U : tick;
    if (tick == 0U && !raised) return 0U;
    unsigned next = start + 1U;
    if (start < 90U && next >= 90U && !raised) next += 120U;
    if (start < 210U && next >= 210U && raised) next -= 120U;
    return next >= 270U ? 0U : next;
}

WaveReference wave_reference(float time) {
    if (!std::isfinite(time) || time < 0.0F) {
        throw std::invalid_argument("invalid wave reference time");
    }
    const auto smooth = [](float value) {
        const float x = std::clamp(value, 0.0F, 1.0F);
        return x * x * x * (10.0F + x * (-15.0F + 6.0F * x));
    };
    WaveReference reference{};
    reference.phase = std::fmod(time / 6.0F, 1.0F);
    const float t = reference.phase * 8.0F;
    const float shift = smooth((t - .8F) / .8F) * (1.0F - smooth((t - 6.4F) / .8F));
    reference.lift = smooth((t - 1.6F) / .8F) * (1.0F - smooth((t - 5.6F) / .8F));
    const float pitch = -.18F * shift;
    for (unsigned leg = 0; leg < 4; ++leg) {
        const float shoulder_x = leg < 2 ? .1934F : -.1934F;
        const float side = leg % 2 == 0 ? 1.0F : -1.0F;
        const float wx = shoulder_x + .03F * shift;
        const float wy = side * .16F - .055F * shift;
        const float wz = .022F - .30F;
        // 与归档 reference.py 相同的逆身体俯仰与屈膝 IK。
        const float x = std::cos(pitch) * wx - std::sin(pitch) * wz - shoulder_x;
        const float y = wy - side * .0465F;
        const float z = std::sin(pitch) * wx + std::cos(pitch) * wz;
        const float lateral = side * .0955F;
        const float down = std::sqrt(std::max(y * y + z * z - lateral * lateral, 1.e-8F));
        const float calf =
            -std::acos(std::clamp((x * x + down * down - 2 * .213F * .213F) / (2 * .213F * .213F), -1.0F, 1.0F));
        reference.position[leg * 3] = std::atan2(y, -z) - std::atan2(lateral, down);
        reference.position[leg * 3 + 1] = std::atan2(-x, down) - calf * .5F;
        reference.position[leg * 3 + 2] = calf;
    }
    constexpr std::array<float, 3> raised{-.10F, -1.50F, -1.5635987755982989F};
    for (unsigned j = 0; j < 3; ++j) {
        reference.position[3 + j] += reference.lift * (raised[j] - reference.position[3 + j]);
    }
    const float phase = std::clamp((t - 2.4F) / 3.2F, 0.0F, 1.0F);
    const float envelope = smooth(phase / .15F) * smooth((1.0F - phase) / .15F);
    reference.position[3] += .28F * envelope * std::sin(kTwoPi * 3.0F * phase);
    return reference;
}

Joints PolicyAdapter::wave_torques(const Joints& target, const RobotState& state) const {
    return wave_torques(target, state, Mode::kWave);
}

Joints PolicyAdapter::wave_torques(const Joints& target, const RobotState& state, Mode mode) const {
    // Isaac Lab 2.3 DCMotor._clip_effort, with the archived wave actuator values.
    const auto& spec = spec_for(mode);
    const float effort_limit = spec.effort_limit;
    const float velocity_limit = spec.velocity_limit;
    const auto gains = policy_gains(mode);
    Joints torque{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const auto& feedback = state.motors.joints[i].feedback;
        if (!std::isfinite(target[i]) || !std::isfinite(feedback.position_rad) ||
            !std::isfinite(feedback.velocity_rad_s)) {
            throw std::invalid_argument("non-finite wave actuator input");
        }
        const auto policy_index = std::find(spec.mapping.begin(), spec.mapping.end(), i) - spec.mapping.begin();
        const float raw = gains.kp[policy_index] * (target[i] - feedback.position_rad) -
            gains.kd[policy_index] * feedback.velocity_rad_s;
        const float velocity = std::clamp(feedback.velocity_rad_s, -2.0F * velocity_limit, 2.0F * velocity_limit);
        const float upper = std::min(effort_limit * (1.0F - velocity / velocity_limit), effort_limit);
        const float lower = std::max(effort_limit * (-1.0F - velocity / velocity_limit), -effort_limit);
        torque[i] = std::clamp(raw, lower, upper);
    }
    return torque;
}

}  // namespace go2

namespace go2 {
using namespace detail;  // NOLINT(build/namespaces)

// ============================================================================
// 构造观测：唯一入口，内部按 mode 分派到各策略的单帧布局。
// ============================================================================

std::vector<float> PolicyAdapter::build_observation(const RobotState& state, VelocityCommand command,
    const Action& previous, Mode mode, float reference_time,
    const std::vector<Joints>& sit_trajectory) const {
    const auto observed_command = command_for_mode(command, mode);
    const auto& defaults = default_joint_position(mode);

    if (mode == Mode::kWalk || mode == Mode::kRun || mode == Mode::kTerrain)
        return observation_frame(mode, state, observed_command, previous, spec_for(mode));
    if (mode == Mode::kSit) return build_sit_observation(sit_trajectory, state, previous, reference_time);
    if (mode == Mode::kForwardJump) {
        // Policy::infer fills the archived reference for the accepted controller frame.
        return build_motion_observation(mode, state, default_joint_position(mode), {}, {1, 0, 0, 0}, previous);
    }
    if (mode == Mode::kWave) {
        const auto reference = wave_reference(reference_time);
        std::vector<float> delta(12);
        for (unsigned i = 0; i < 12; ++i) delta[i] = reference.position[i] - defaults[i];
        return observation_frame(mode, state, {}, previous, spec_for(mode),
            {{"reference_delta", delta},
                {"reference_phase", {std::sin(kTwoPi * reference.phase), std::cos(kTwoPi * reference.phase)}}});
    }
    if (mode == Mode::kRightFlip) {
        const Joints reference{};
        return build_motion_observation(mode, state, reference, reference, state.imu.six_axis_quaternion, previous);
    }

    throw std::invalid_argument("unsupported observation adapter");
}

// ============================================================================
// class Policy：模型加载、统一推理入口与可用性查询。
// ============================================================================

Quaternion motion_yaw_alignment(const Quaternion& actual, const Quaternion& reference) {
    return multiply(yaw_quaternion(actual), conjugate(yaw_quaternion(reference)));
}

std::vector<float> PolicyAdapter::build_motion_observation(Mode mode, const RobotState& state,
    const Joints& reference_position, const Joints& reference_velocity, const Quaternion& reference_quaternion,
    const Action& previous, const Quaternion& right_yaw_alignment) const {
    if (mode != Mode::kRightFlip && mode != Mode::kForwardJump) {
        throw std::invalid_argument("motion observation requires a motion mode");
    }
    const auto aligned_reference = multiply(right_yaw_alignment, reference_quaternion);
    // Slot 11 Isaac Lab uses R_robot^T R_reference, first two columns in
    // row-major order. Preserve slot 18's distinct historical contract.
    const auto relative = mode == Mode::kForwardJump
        ? multiply(conjugate(state.imu.six_axis_quaternion), aligned_reference)
        : multiply(conjugate(aligned_reference), state.imu.six_axis_quaternion);
    auto orientation = right_flip_rotation_six(relative);
    if (mode == Mode::kForwardJump) {
        const auto [w, x, y, z] = normalize(relative);
        orientation = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
            2 * (x * z - y * w), 2 * (y * z + x * w)};
    }
    return observation_frame(mode, state, {}, previous, spec_for(mode),
        {{"reference_position", {reference_position.begin(), reference_position.end()}},
            {"reference_velocity", {reference_velocity.begin(), reference_velocity.end()}},
            {"reference_orientation", {orientation.begin(), orientation.end()}}});
}

}  // namespace go2
