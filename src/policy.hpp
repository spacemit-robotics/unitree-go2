/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef POLICY_HPP
#define POLICY_HPP
#include "policy_adapter.hpp"
#include <map>
#include <memory>
#include <mutex>

namespace rl_policy {
class PolicyExecutor;
}

namespace go2 {
class Policy final {
public:
    explicit Policy(const std::string& model_directory, const std::string& config_path = "",
        std::shared_ptr<const PolicyAdapter> adapter = nullptr);
    ~Policy();
    // ONNX inference through model_zoo/rl; state and frame drive motion references.
    Action infer(Mode mode, const std::vector<float>& observation, const RobotState* state = nullptr,
        std::uint64_t policy_frame = std::numeric_limits<std::uint64_t>::max());
    void reset_mode(Mode mode, const RobotState* state = nullptr);
    // A mode is available only after all of its model variants load successfully.
    bool has_model(Mode mode) const;
    const std::string& load_error(Mode mode) const;

private:
    std::shared_ptr<const PolicyAdapter> adapter_;
    std::mutex mutex_;
    std::map<Mode, rl_policy::PolicyExecutor*> models_;
    std::map<Mode, std::string> load_errors_;
    std::map<Mode, std::uint64_t> steps_;
    std::map<Mode, Quaternion> yaw_alignments_, initial_reference_quaternions_;
    std::map<Mode, std::vector<MotionReference>> motion_references_;
};
}  // namespace go2

#endif  // POLICY_HPP
