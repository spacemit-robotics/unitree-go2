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
    assert(argc == 2);
    using go2::Mode;

    auto state = fresh(1.0);
    go2::Action previous{};
    seed_policy_state(state, previous);

    const auto flat =
            test_adapter().build_observation(state, {2.0F, -2.0F, 3.0F}, previous,
                                                            Mode::kWalk);
    assert(flat.size() == 45U);
    near(flat[0], 0.2F);
    near(flat[1], -0.4F);
    near(flat[2], 0.6F);
    near(flat[3], 0.0F);
    near(flat[4], 0.0F);
    near(flat[5], -1.0F);
    near(flat[6], 1.0F);
    near(flat[7], -0.4F);
    near(flat[8], 1.0F);
    check_proprioception(flat, 9U, state, previous, Mode::kWalk,
                                                go2::kIsaacLabDefaultJointPos);

    const auto flat_lower =
            test_adapter().build_observation(state, {-2.0F, 2.0F, -3.0F}, previous,
                                                            Mode::kWalk);
    near(flat_lower[6], -1.0F);
    near(flat_lower[7], 0.4F);
    near(flat_lower[8], -1.0F);

    const auto running =
            test_adapter().build_observation(state, {9.0F, -3.0F, 4.0F}, previous,
                                                            Mode::kRun);
    assert(running.size() == 45U);
    near(running[0], 0.25F);
    near(running[1], -0.5F);
    near(running[2], 0.75F);
    near(running[6], 6.0F);  // Forward command clamps to 3 m/s, then scales by 2.
    near(running[7], -1.0F);
    near(running[8], 0.5F);
    check_proprioception(running, 9U, state, previous, Mode::kRun,
                                                go2::kIsaacGymDefaultJointPos);

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

    const auto nominal = go2::wave_reference(0.0F).position;
    const auto wave = test_adapter().build_observation(state, {}, previous, Mode::kWave, 2.0F);
    assert(wave.size() == 56U);
    near(wave[0], .25F);
    check_proprioception(wave, 6U, state, previous, Mode::kWave, nominal);
    near(wave[54], std::sin(go2::kTwoPi / 3.0F));
    near(wave[55], std::cos(go2::kTwoPi / 3.0F));
    const auto reference = go2::wave_reference(2.0F);
    for (std::size_t i = 0; i < 12U; ++i) near(wave[42U + i], reference.position[i] - nominal[i]);
    near(nominal[0], .06395703554F);
    near(nominal[1], .83890676498F);
    near(nominal[2], -1.67781352997F);

    assert(go2::next_wave_tick(0U, false) == 0U);
    assert(go2::next_wave_tick(0U, true) == 31U);
    assert(go2::next_wave_tick(89U, false) == 210U);
    assert(go2::next_wave_tick(209U, true) == 90U);
    assert(go2::next_wave_tick(269U, false) == 0U);
    go2::Action residual{};
    residual.fill(2.0F);
    const auto wave_target = test_adapter().action_to_hardware_targets(residual, Mode::kWave, 2.0F);
    near(wave_target[go2::kLegMajorPolicyToHardware[3]], reference.position[3] + .1F);
    near(wave_target[go2::kLegMajorPolicyToHardware[4]], reference.position[4]);
    near(wave_target[go2::kLegMajorPolicyToHardware[5]], reference.position[5]);
    auto motor_state = fresh(1.0);
    go2::Joints far_target{};
    far_target.fill(100.0F);
    for (auto& joint : motor_state.motors.joints) joint.feedback.velocity_rad_s = 15.0F;
    for (float torque : test_adapter().wave_torques(far_target, motor_state)) near(torque, 11.75F);
    for (auto& joint : motor_state.motors.joints) joint.feedback.velocity_rad_s = 45.0F;
    for (float torque : test_adapter().wave_torques(far_target, motor_state)) near(torque, -11.75F);
    far_target.fill(-100.0F);
    for (auto& joint : motor_state.motors.joints) joint.feedback.velocity_rad_s = -45.0F;
    for (float torque : test_adapter().wave_torques(far_target, motor_state)) near(torque, 11.75F);

    const go2::Joints reference_position{};
    const go2::Joints reference_velocity{};
    const auto alignment = go2::motion_yaw_alignment(yaw(0.7F), yaw(0.2F));
    auto right_state = state;
    right_state.imu.six_axis_quaternion = yaw(0.7F);
    const auto right = test_adapter().build_motion_observation(
            Mode::kRightFlip, right_state, reference_position, reference_velocity,
            yaw(0.2F), previous, alignment);
    assert(right.size() == 69U);
    near(right[24], 1.0F);
    near(right[25], 0.0F);
    near(right[26], 0.0F);
    near(right[27], 1.0F);
    near(right[28], 0.0F);
    near(right[29], 0.0F);

    go2::Action zero{};
    for (const auto mode : {Mode::kWalk, Mode::kRun, Mode::kTerrain, Mode::kWave,
                                                    Mode::kRightFlip}) {
        const auto target = test_adapter().action_to_hardware_targets(zero, mode);
        const auto& map = test_adapter().policy_to_hardware(mode);
        const go2::Joints* defaults = &go2::kIsaacLabDefaultJointPos;
        if (mode == Mode::kRun || mode == Mode::kTerrain)
            defaults = &go2::kIsaacGymDefaultJointPos;
        if (mode == Mode::kWave) defaults = &nominal;
        if (mode == Mode::kRightFlip) defaults = &go2::kRightFlipDefaultJointPos;
        for (std::size_t policy_index = 0; policy_index < 12U; ++policy_index) {
            near(target[map[policy_index]], (*defaults)[policy_index]);
        }
    }

    for (float gain : test_adapter().policy_gains(Mode::kWalk).kd) near(gain, 0.5F);
    for (const auto mode : {Mode::kRun, Mode::kTerrain}) {
        for (float gain : test_adapter().policy_gains(mode).kp) near(gain, 20.0F);
        for (float gain : test_adapter().policy_gains(mode).kd) near(gain, 0.5F);
    }

    rejects([&] {
        test_adapter().limit_command(
                {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F});
    });
    auto invalid_action = zero;
    invalid_action[0] = std::numeric_limits<float>::infinity();
    rejects([&] { test_adapter().action_to_hardware_targets(invalid_action, Mode::kRun); });

    go2::Policy policy(argv[1], GO2_TEST_POLICY_CONFIG, test_adapter_ptr());
    struct SmokeCase {
        Mode mode;
        std::size_t dimension;
    };
    const std::array<SmokeCase, 4> cases{{
        {Mode::kWalk, 45U}, {Mode::kRun, 225U},
        {Mode::kTerrain, 225U}, {Mode::kWave, 56U},
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
    for (const auto mode : {Mode::kWalk, Mode::kRun, Mode::kTerrain}) {
        for (const float accepted : {.37F, -.21F, 0.F}) {
            go2::Action accepted_action{};
            accepted_action.fill(accepted);
            const auto obs = test_adapter().build_observation(state, {}, accepted_action, mode);
            check_proprioception(obs, 9U, state, accepted_action, mode,
                mode == Mode::kWalk ? go2::kIsaacLabDefaultJointPos : go2::kIsaacGymDefaultJointPos);
        }
    }
    // Replay the right-flip reference sequence twice: reset must reproduce frame 0.
    go2::Action first{};
    for (int episode = 0; episode < 2; ++episode) {
        policy.reset_mode(Mode::kRightFlip, &state);
        for (std::uint64_t frame = 0; frame < 200U; ++frame) {
            const auto obs = test_adapter().build_observation(state, {}, {}, Mode::kRightFlip);
            const auto action = policy.infer(Mode::kRightFlip, obs, &state, frame);
            for (float v : action) assert(std::isfinite(v));
            if (frame == 0U) {
                if (episode == 0) first = action;
                else for (std::size_t i = 0; i < 12; ++i) near(action[i], first[i]);
            }
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
    auto bad_observation = std::vector<float>(45U, 0.0F);
    bad_observation[4] = std::numeric_limits<float>::quiet_NaN();
    rejects([&] { policy.infer(Mode::kWalk, bad_observation); });

    std::cout << "PASS: model contracts, mappings, gains, ONNX episodes\n";
}
