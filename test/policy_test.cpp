/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "controller.hpp"
#include "policy.hpp"
#include "support.hpp"

#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>

namespace {

using go2::Mode;

void seed_policy_state(go2::RobotState& state, go2::Action& previous) {
    state.imu.angular_velocity = {1.0F, -2.0F, 3.0F};
    for (std::size_t hardware_index = 0; hardware_index < go2::kNumJoints;
                ++hardware_index) {
        state.motors.joints[hardware_index].feedback.position_rad =
                0.05F * static_cast<float>(hardware_index);
        state.motors.joints[hardware_index].feedback.velocity_rad_s =
                static_cast<float>(hardware_index);
        previous[hardware_index] = -0.03F * static_cast<float>(hardware_index);
    }
}

void check_proprioception(const std::vector<float>& observation,
                                                    std::size_t offset, const go2::RobotState& state,
                                                    const go2::Action& previous, go2::Mode mode,
                                                    const go2::Joints& defaults) {
    const auto& map = test_adapter().policy_to_hardware(mode);
    for (std::size_t policy_index = 0; policy_index < go2::kNumJoints;
                ++policy_index) {
        const auto hardware_index = map[policy_index];
        near(observation[offset + policy_index],
                    state.motors.joints[hardware_index].feedback.position_rad -
                            defaults[policy_index]);
        near(observation[offset + 12U + policy_index],
                    state.motors.joints[hardware_index].feedback.velocity_rad_s * 0.05F);
        near(observation[offset + 24U + policy_index], previous[policy_index]);
    }
}

go2::Quaternion yaw(float radians) {
    return {std::cos(0.5F * radians), 0.0F, 0.0F,
                    std::sin(0.5F * radians)};
}

void print_wave_probe() {
    std::cout << std::setprecision(9);
    for (unsigned frame = 0; frame <= 600; ++frame) {
        const float time = frame * .01F;
        const auto reference = go2::wave_reference(time);
        std::cout << time;
        for (float value : reference.position) std::cout << ' ' << value;
        std::cout << ' ' << reference.lift << ' ' << reference.phase << '\n';
    }
}

void print_sit_probe(const std::string& reference) {
    const auto trajectory = go2::load_sit_reference(reference);
    std::cout << std::setprecision(9);
    auto state = fresh(1);
    go2::Action action{}, previous{};
    seed_policy_state(state, previous);
    state.imu.six_axis_quaternion = {.9238795F, 0, .3826834F, 0};
    for (unsigned i = 0; i < 12; ++i) action[i] = (static_cast<float>(i) - 6) * .3F;
    for (float exit : {1.F, 4.F, 180.F, std::numeric_limits<float>::infinity()}) {
        for (unsigned tick = 0; tick <= 500; ++tick) {
            const float time = tick * .02F + (tick % 2 ? .007F : 0.F) + (exit == 180.F ? 178.F : 0.F);
            const auto ref = go2::sit_reference(trajectory, time, exit);
            const auto obs = test_adapter().build_sit_observation(trajectory, state, previous, time, exit);
            const auto target = test_adapter().sit_hardware_targets(trajectory, action, time, exit);
            std::cout << exit << ' ' << time;
            for (float v : ref.position) std::cout << ' ' << v;
            std::cout << ' ' << ref.seated << ' ' << ref.rising;
            for (float v : obs) std::cout << ' ' << v;
            for (float v : target) std::cout << ' ' << v;
            std::cout << '\n';
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--sit-probe") {
        print_sit_probe(argv[2]);
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--wave-probe") {
        print_wave_probe();
        return 0;
    }
    assert(argc == 2);
    using go2::Mode;

    auto state = fresh(1.0);
    go2::Action previous{};
    seed_policy_state(state, previous);

    const auto walking = test_adapter().build_observation(state, {9.F, -3.F, 4.F}, previous, Mode::kWalk);
    assert(walking.size() == 45U);
    near(walking[0], .2F);
    near(walking[1], -.4F);
    near(walking[2], .6F);
    near(walking[6], 1.F);
    near(walking[7], -.4F);
    near(walking[8], 1.F);
    const go2::Joints walking_default{.1F, -.1F, .1F, -.1F, .8F, .8F, 1.F, 1.F,
        -1.5F, -1.5F, -1.5F, -1.5F};
    check_proprioception(walking, 9U, state, previous, Mode::kWalk, walking_default);
    for (float gain : test_adapter().policy_gains(Mode::kWalk).kp) near(gain, 25.F);
    for (float gain : test_adapter().policy_gains(Mode::kWalk).kd) near(gain, .5F);

    const auto waving = test_adapter().build_observation(state, {}, previous, Mode::kWave, 2.F);
    assert(waving.size() == 56U);
    near(waving[0], .25F);
    near(waving[1], -.5F);
    near(waving[2], .75F);
    const auto& wave_spec = test_adapter().spec_for(Mode::kWave);
    check_proprioception(waving, 6U, state, previous, Mode::kWave, wave_spec.default_pos);
    const auto reference = go2::wave_reference(2.F);
    for (unsigned i = 0; i < 12; ++i)
        near(waving[42+i], reference.position[i] - wave_spec.default_pos[i]);
    go2::Action wave_action{};
    wave_action.fill(1.F);
    const auto wave_target = test_adapter().action_to_hardware_targets(wave_action, Mode::kWave, 2.F);
    near(wave_target[wave_spec.mapping[3]], reference.position[3]+.1F);
    near(wave_target[wave_spec.mapping[4]], reference.position[4]);
    near(wave_target[wave_spec.mapping[5]], reference.position[5]);
    // Terrain holds the command in physical units and normalizes it once with the
    // trained [2, 2, 0.25] scale; {9, -3, 4} clamps to {1.0, -0.5, 2.0}.
    const auto terrain =
            test_adapter().build_observation(state, {9.0F, -3.0F, 4.0F}, previous,
                                                            Mode::kTerrain);
    assert(terrain.size() == 45U);
    near(terrain[0], 0.25F);
    near(terrain[1], -0.5F);
    near(terrain[2], 0.75F);
    near(terrain[6], 2.0F);
    near(terrain[7], -1.0F);
    near(terrain[8], 0.5F);
    check_proprioception(terrain, 9U, state, previous, Mode::kTerrain,
                                                go2::kIsaacGymDefaultJointPos);

    auto motor_state = fresh(1.0);
    go2::Joints far_target{};
    far_target.fill(100.0F);
    for (auto& joint : motor_state.motors.joints) joint.feedback.velocity_rad_s = 15.0F;
    for (float torque : test_adapter().policy_torques(far_target, motor_state, Mode::kWave)) near(torque, 11.75F);
    for (auto& joint : motor_state.motors.joints) joint.feedback.velocity_rad_s = 45.0F;
    for (float torque : test_adapter().policy_torques(far_target, motor_state, Mode::kWave)) near(torque, -11.75F);
    far_target.fill(-100.0F);
    for (auto& joint : motor_state.motors.joints) joint.feedback.velocity_rad_s = -45.0F;
    for (float torque : test_adapter().policy_torques(far_target, motor_state, Mode::kWave)) near(torque, 11.75F);

    const go2::Joints reference_position{};
    const go2::Joints reference_velocity{};
    const auto alignment = go2::motion_yaw_alignment(yaw(0.7F), yaw(0.2F));
    auto right_state = state;
    right_state.imu.six_axis_quaternion = yaw(0.7F);
    const auto right = test_adapter().build_motion_observation(
            Mode::kForwardJump, right_state, reference_position, reference_velocity,
            yaw(0.2F), previous, alignment);
    assert(right.size() == 69U);
    near(right[24], 1.0F);
    near(right[25], 0.0F);
    near(right[26], 0.0F);
    near(right[27], 1.0F);
    near(right[28], 0.0F);
    near(right[29], 0.0F);

    go2::Action zero{};
    for (const auto mode : {Mode::kTerrain}) {
        const auto target = test_adapter().action_to_hardware_targets(zero, mode);
        const auto& map = test_adapter().policy_to_hardware(mode);
        const go2::Joints* defaults = &go2::kIsaacGymDefaultJointPos;
        for (std::size_t policy_index = 0; policy_index < 12U; ++policy_index) {
            near(target[map[policy_index]], (*defaults)[policy_index]);
        }
    }

    for (const auto mode : {Mode::kTerrain}) {
        for (float gain : test_adapter().policy_gains(mode).kp) near(gain, 20.0F);
        for (float gain : test_adapter().policy_gains(mode).kd) near(gain, 0.5F);
    }

    rejects([&] {
        test_adapter().build_observation(state,
                {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F}, {}, Mode::kTerrain);
    });
    auto invalid_action = zero;
    invalid_action[0] = std::numeric_limits<float>::infinity();
    rejects([&] { test_adapter().action_to_hardware_targets(invalid_action, Mode::kWave); });

    go2::Policy policy(argv[1], GO2_TEST_POLICY_CONFIG, test_adapter_ptr());
    struct SmokeCase {
        Mode mode;
        std::size_t dimension;
    };
    const std::array<SmokeCase, 4> cases{{
        {Mode::kWalk, 45U}, {Mode::kTerrain, 225U}, {Mode::kWave, 56U}, {Mode::kSit, 56U},
    }};
    for (const auto& test : cases) {
        std::cout << "smoke " << go2::mode_name(test.mode) << std::endl;
        policy.reset_mode(test.mode, &state);
        std::vector<float> observation(test.dimension, 0.0F);
        const auto action = policy.infer(test.mode, observation, &state, 0U);
        for (float value : action) assert(std::isfinite(value));
    }
    // Inferred results may be rejected by the controller. Assembly must use the
    // supplied accepted action, including zero after a mode reset, in every frame.
    for (const auto mode : {Mode::kTerrain}) {
        for (const float accepted : {.37F, -.21F, 0.F}) {
            go2::Action accepted_action{};
            accepted_action.fill(accepted);
            const auto obs = test_adapter().build_observation(state, {}, accepted_action, mode);
            check_proprioception(obs, 9U, state, accepted_action, mode,
                go2::kIsaacGymDefaultJointPos);
        }
    }
    {
        const auto sit_trajectory = go2::load_sit_reference(
            std::string(GO2_TEST_APP) + "/resources/motions/12.go2_sit.motion");
        // Exercise real ONNX files, including all jump reference frames and reset.
        auto state = fresh(1);
        for (auto mode : {Mode::kForwardJump, Mode::kSit}) {
            go2::Action first{}, previous{};
            for (unsigned episode = 0; episode < 2; ++episode) {
                previous = {};
                policy.reset_mode(mode, &state);
                for (unsigned n = 0; n < (mode == Mode::kSit ? 600U : 120U); ++n) {
                    auto obs = mode == Mode::kSit
                        ? test_adapter().build_sit_observation(sit_trajectory, state, previous, n * .02F, 6.F)
                        : test_adapter().build_observation(state, {}, previous, mode, n * .02F);
                    auto action = policy.infer(mode, obs, &state, n);
                    for (float v : action)
                        assert(std::isfinite(v));
                    if (n == 0) {
                        if (!episode)
                            first = action;
                        else
                            for (unsigned i = 0; i < 12; ++i)
                                near(action[i], first[i]);
                    }
                    previous = action;
                }
            }
        }
        // A nonzero relative yaw catches row/column and inverse-rotation mistakes.
        const auto obs = test_adapter().build_motion_observation(
            Mode::kForwardJump, state, {}, {}, {std::cos(.3F), 0, 0, std::sin(.3F)}, {});
        near(obs[24], std::cos(.6F));
        near(obs[25], -std::sin(.6F));
        near(obs[26], std::sin(.6F));
        near(obs[27], std::cos(.6F));
        near(obs[28], 0);
        near(obs[29], 0);
    }
    far_target.fill(100.F);
    for (float value : test_adapter().policy_torques(far_target, fresh(1), Mode::kSit)) near(value, 23.5F);
    auto bad_observation = std::vector<float>(225U, 0.0F);
    bad_observation[4] = std::numeric_limits<float>::quiet_NaN();
    rejects([&] { policy.infer(Mode::kTerrain, bad_observation); });

    std::cout << "PASS: model contracts, mappings, gains, ONNX episodes\n";
}
