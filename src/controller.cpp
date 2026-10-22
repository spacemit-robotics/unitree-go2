#include "controller.hpp"
#include "policy.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace go2 {
namespace {
enum class HistoryOrder { kOldestFirst, kNewestFirst };

template <std::size_t Frames, std::size_t Size>
void seed_history(std::array<float, Size> &history, const std::vector<float> &frame) {
    static_assert(Frames > 0 && Size % Frames == 0);
    if (frame.size() != Size / Frames)
        throw std::logic_error("policy history frame dimension mismatch");
    for (std::size_t index = 0; index < Frames; ++index)
        std::copy(frame.begin(), frame.end(), history.begin() + index * frame.size());
}

template <std::size_t Frames, std::size_t Size>
void update_history(
    std::array<float, Size> &history, bool &initialized, const std::vector<float> &frame, HistoryOrder order) {
    static_assert(Frames > 0 && Size % Frames == 0);
    if (frame.size() != Size / Frames)
        throw std::logic_error("policy history frame dimension mismatch");
    if (!initialized) {
        seed_history<Frames>(history, frame);
        initialized = true;
    } else if (order == HistoryOrder::kNewestFirst) {
        std::move_backward(history.begin(), history.end() - frame.size(), history.end());
        std::copy(frame.begin(), frame.end(), history.begin());
    } else {
        std::move(history.begin() + frame.size(), history.end(), history.begin());
        std::copy(frame.begin(), frame.end(), history.end() - frame.size());
    }
}

Joints positions(const RobotState &state) {
    Joints result{};
    for (std::size_t i = 0; i < result.size(); ++i)
        result[i] = state.motors.joints[i].feedback.position_rad;
    return result;
}
Joints interpolate(const Joints &from, const Joints &to, double elapsed, double duration) {
    // Same single-precision smoothstep as the original stand/crouch implementation.
    const float t = std::clamp(static_cast<float>(elapsed) / static_cast<float>(duration), 0.F, 1.F);
    const float a = t * t * (3.F - 2.F * t);
    Joints result{};
    for (std::size_t i = 0; i < result.size(); ++i)
        result[i] = from[i] * (1.F - a) + to[i] * a;
    return result;
}
// The flip episodes are aerial: while one is running, a missing or faulted
// policy frame and a body-IMU dropout must hold the last verified command rather
// than command a crouch.  The same rule as the aerial envelope: real faults
// (invalid/stale motor feedback, driver error, temperature) still stop the
// episode, but nothing may actively drive the robot into the ground mid-air.
bool aerial_flip(Mode mode) {
    return mode == Mode::kForwardJump;
}

Commands pose_commands(const PolicyAdapter &adapter, const Joints &target, bool walking, Mode mode = Mode::kStand,
    std::size_t variant_index = 0) {
    Commands result{};
    // Pose control has no policy model, including the FixStand entry stage.
    const GainSet gains = walking ? adapter.policy_gains(mode, variant_index) : GainSet{kStandKp, kStandKd};
    const auto &kp = gains.kp;
    const auto &kd = gains.kd;
    if (walking) {
        // Only the mode gain arrays are model-side (slot order); the joint target
        // is already in this robot's own motor order.
        const auto &map = adapter.variant(mode, variant_index).mapping;
        for (std::size_t policy_index = 0; policy_index < result.size(); ++policy_index) {
            const auto hardware_index = map[policy_index];
            result[hardware_index].mode = MotorMode::kFoc;
            result[hardware_index].position_rad = target[hardware_index];
            result[hardware_index].kp = kp[policy_index];
            result[hardware_index].kd = kd[policy_index];
        }
    } else {
        for (std::size_t hardware_index = 0; hardware_index < result.size(); ++hardware_index) {
            result[hardware_index].mode = MotorMode::kFoc;
            result[hardware_index].position_rad = target[hardware_index];
            result[hardware_index].kp = kp[hardware_index];
            result[hardware_index].kd = kd[hardware_index];
        }
    }
    return result;
}

float slew(float current, float target, double acceleration_delta, double deceleration_delta) {
    // A reversal brakes to zero before building speed in the other direction.
    if (current * target < 0.0F)
        target = 0.0F;
    const float delta = target - current;
    const float limit =
        static_cast<float>(std::fabs(target) < std::fabs(current) ? deceleration_delta : acceleration_delta);
    return current + std::clamp(delta, -limit, limit);
}

// The v9 "翻身" action is a deterministic, low-level pose sequence rather
// than an ONNX policy. Each 0.8 s segment blends between recorded poses,
// starting with Pose A and ending at the configured mechanical Pose A.
constexpr float kRolloverPi = 3.14159265358979323846F;
constexpr float kRolloverTiltThreshold = 60.0F * kRolloverPi / 180.0F;
constexpr double kRolloverSegmentDuration = 0.8;
// Six uses poses 1-4, eight uses poses 1-6, and nine uses poses 1-7.
// Eight omits the original segment 8 and returns directly from pose 6 to Pose A.
constexpr std::size_t kRolloverSegmentCount = 9;
constexpr float rollover_rad(float degrees) { return degrees * kRolloverPi / 180.0F; }

bool tilt_exceeds(const ImuSample &imu, float roll_threshold, float pitch_threshold) {
    const auto &q = imu.quaternion;
    const float norm = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (!std::isfinite(norm) || norm <= 1e-6F)
        return false;
    const float roll = std::atan2(2.0F * (q[0] * q[1] + q[2] * q[3]), norm - 2.0F * (q[1] * q[1] + q[2] * q[2]));
    const float pitch = std::asin(std::clamp(2.0F * (q[0] * q[2] - q[3] * q[1]) / norm, -1.0F, 1.0F));
    return std::fabs(roll) > roll_threshold || std::fabs(pitch) > pitch_threshold;
}
bool rollover_tilted(const ImuSample &imu) {
    return tilt_exceeds(imu, kRolloverTiltThreshold, kRolloverTiltThreshold);
}
constexpr std::array<std::array<float, kNumJoints>, 7> kRolloverRecordedPoses{{
    {rollover_rad(-12.41F), rollover_rad(180.69F), rollover_rad(-157.62F), rollover_rad(48.39F), rollover_rad(92.19F),
        rollover_rad(-156.62F), rollover_rad(-12.41F), rollover_rad(180.69F), rollover_rad(-157.62F),
        rollover_rad(48.39F), rollover_rad(92.19F), rollover_rad(-156.62F)},
    {rollover_rad(-16.56F), rollover_rad(165.84F), rollover_rad(-148.77F), rollover_rad(48.42F), rollover_rad(145.70F),
        rollover_rad(-135.56F), rollover_rad(-16.56F), rollover_rad(165.84F), rollover_rad(-148.77F),
        rollover_rad(48.42F), rollover_rad(145.70F), rollover_rad(-135.56F)},
    {rollover_rad(-42.57F), rollover_rad(173.07F), rollover_rad(-148.89F), rollover_rad(0.61F), rollover_rad(145.73F),
        rollover_rad(-135.94F), rollover_rad(-42.57F), rollover_rad(173.07F), rollover_rad(-148.89F),
        rollover_rad(0.61F), rollover_rad(145.73F), rollover_rad(-135.94F)},
    {rollover_rad(-0.86F), rollover_rad(137.94F), rollover_rad(-152.15F), rollover_rad(-6.05F), rollover_rad(138.46F),
        rollover_rad(-152.04F), rollover_rad(-0.86F), rollover_rad(137.94F), rollover_rad(-152.15F),
        rollover_rad(-6.05F), rollover_rad(138.46F), rollover_rad(-152.04F)},
    {rollover_rad(-24.41F), rollover_rad(164.64F), rollover_rad(-150.95F), rollover_rad(-47.62F), rollover_rad(141.23F),
        rollover_rad(-149.55F), rollover_rad(-24.41F), rollover_rad(164.64F), rollover_rad(-150.95F),
        rollover_rad(-47.62F), rollover_rad(141.23F), rollover_rad(-149.55F)},
    {rollover_rad(22.38F), rollover_rad(29.45F), rollover_rad(-123.36F), rollover_rad(-48.56F), rollover_rad(136.08F),
        rollover_rad(-148.25F), rollover_rad(22.38F), rollover_rad(29.45F), rollover_rad(-123.36F),
        rollover_rad(-48.56F), rollover_rad(136.08F), rollover_rad(-148.25F)},
    {rollover_rad(22.38F), rollover_rad(29.45F), rollover_rad(-123.36F), rollover_rad(-16.07F), rollover_rad(149.03F),
        rollover_rad(-156.68F), rollover_rad(22.38F), rollover_rad(29.45F), rollover_rad(-123.36F),
        rollover_rad(-16.07F), rollover_rad(149.03F), rollover_rad(-156.68F)},
}};
Joints rollover_target(const Joints &initial, const Joints &pose_a, double elapsed_seconds) {
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0)
        return initial;
    double remaining = elapsed_seconds;
    std::size_t segment = 0;
    while (segment + 1 < kRolloverSegmentCount && remaining >= kRolloverSegmentDuration) {
        remaining -= kRolloverSegmentDuration;
        ++segment;
    }
    const float alpha = std::clamp(static_cast<float>(remaining / kRolloverSegmentDuration), 0.0F, 1.0F);
    const float smooth = alpha * alpha * (3.0F - 2.0F * alpha);
    const Joints &from = segment == 0 ? initial : (segment == 1 ? pose_a : kRolloverRecordedPoses[segment - 2]);
    const Joints &to =
        segment == 0 ? pose_a : (segment == kRolloverSegmentCount - 1 ? pose_a : kRolloverRecordedPoses[segment - 1]);
    Joints result{};
    for (std::size_t i = 0; i < kNumJoints; ++i)
        result[i] = from[i] * (1.0F - smooth) + to[i] * smooth;
    return result;
}

bool requires_fixstand(Mode mode) {
    switch (mode) {
    case Mode::kWalk:
    case Mode::kRun:
    case Mode::kForwardJump:
    case Mode::kSit:
    case Mode::kTerrain:
    case Mode::kWave:
    case Mode::kRightFlip:
        return true;
    default:
        return false;
    }
}

Joints action_entry_target(const PolicyAdapter &adapter, Mode mode, const std::vector<Joints> &sit_trajectory) {
    switch (mode) {
    case Mode::kWave:
    case Mode::kForwardJump:
    case Mode::kSit:
        return adapter.action_to_hardware_targets({}, mode, 0.F, sit_trajectory);
    default:
        return kStandPose;
    }
}

} // namespace

const char *mode_name(Mode mode) {
    switch (mode) {
    case Mode::kPassive:
        return "passive";
    case Mode::kCrouch:
        return "crouch";
    case Mode::kStand:
        return "stand";
    case Mode::kWalk:
        return "walk";
    case Mode::kRun:
        return "run";
    case Mode::kTerrain:
        return "terrain";
    case Mode::kForwardJump:
        return "forward_jump";
    case Mode::kSit:
        return "sit";
    case Mode::kWave:
        return "wave";
    case Mode::kRightFlip:
        return "right_flip";
    case Mode::kRollover:
        return "rollover";
    }
    return "unknown";
}
Mode parse_mode(const std::string &value) {
    if (value == "passive")
        return Mode::kPassive;
    if (value == "crouch")
        return Mode::kCrouch;
    if (value == "stand")
        return Mode::kStand;
    if (value == "walk")
        return Mode::kWalk;
    if (value == "run")
        return Mode::kRun;
    if (value == "terrain")
        return Mode::kTerrain;
    if (value == "forward_jump")
        return Mode::kForwardJump;
    if (value == "sit")
        return Mode::kSit;
    if (value == "wave")
        return Mode::kWave;
    if (value == "right_flip")
        return Mode::kRightFlip;
    if (value == "rollover")
        return Mode::kRollover;
    if (value == "navigation" || value == "ai_agent")
        return Mode::kWalk;
    throw std::invalid_argument("unknown mode");
}

Controller::Controller(ControllerConfig config) : config_(std::move(config)) {
    if (!config_.policy_adapter)
        throw std::invalid_argument("missing policy adapter configuration");
    for (double value : {config_.motor_soft_timeout, config_.motor_hard_timeout, config_.rr_motor_soft_timeout,
                config_.rr_motor_hard_timeout, config_.imu_soft_timeout, config_.imu_hard_timeout, config_.command_timeout,
                config_.stand_duration, config_.crouch_pose_duration, config_.crouch_duration,
                config_.move_linear_acceleration, config_.move_yaw_acceleration, config_.move_linear_deceleration,
                config_.move_yaw_deceleration,
                config_.run_linear_acceleration, config_.run_yaw_acceleration, config_.run_linear_deceleration,
                config_.run_yaw_deceleration, static_cast<double>(config_.maximum_position),
                static_cast<double>(config_.maximum_velocity)}) {
        if (!std::isfinite(value) || value <= 0)
            throw std::invalid_argument("control limits must be finite and positive");
    }
    if (config_.motor_hard_timeout <= config_.motor_soft_timeout || config_.motor_hard_timeout > .5 ||
        config_.rr_motor_hard_timeout <= config_.rr_motor_soft_timeout || config_.rr_motor_hard_timeout > 3.0 ||
        config_.imu_hard_timeout <= config_.imu_soft_timeout || config_.maximum_temperature <= 0 ||
        config_.stand_duration < .2 || config_.stand_duration > 5 || config_.crouch_pose_duration < .2 ||
        config_.crouch_pose_duration > 10 || config_.crouch_duration < .2 || config_.crouch_duration > 5) {
        throw std::invalid_argument("invalid timeout ordering, temperature or transition duration");
    }
    for (float value : config_.crouch_pose)
        if (!std::isfinite(value))
            throw std::invalid_argument("invalid crouch pose");
}

bool mode_uses_policy(Mode mode) {
    return mode != Mode::kPassive && mode != Mode::kStand && mode != Mode::kCrouch && mode != Mode::kRollover;
}

StatusLightState status_light_state(Mode mode, const RobotState& state, TimePoint now, bool input_alive,
    double imu_timeout, double motor_timeout) {
    // Transaction failures and individual missing replies deliberately do not
    // feed this indicator. Only loss of every motor's feedback is a bus fault.
    if (state.motors.startup_pose_invalid || state.motors.session_anchor_faulted)
        return {StatusLight::kRed, "startup_pose_invalid"};
    if (!state.imu.valid || state.imu.received_at == TimePoint{} || now < state.imu.received_at ||
        seconds(now - state.imu.received_at) > imu_timeout)
        return {StatusLight::kRed, "imu_no_data"};
    if (!input_alive)
        return {StatusLight::kRed, "teleop_no_data"};
    const bool all_offline = std::all_of(state.motors.joints.begin(), state.motors.joints.end(),
        [now, motor_timeout](const TimedJointFeedback& joint) {
            return !joint.valid || joint.received_at == TimePoint{} || now < joint.received_at ||
                seconds(now - joint.received_at) > motor_timeout;
        });
    if (all_offline)
        return {StatusLight::kRed, "all_motors_offline"};
    return {mode_uses_policy(mode) ? StatusLight::kOrange : StatusLight::kGreen, ""};
}

Controller::Health Controller::motor_health(
    const RobotState &s, TimePoint now, std::string &reason, bool enforce_position_limits) const {
    if (s.motors.session_anchor_faulted) {
        reason = "session anchor fault; restart at Pose A";
        return Health::kUnsafe;
    }
    // Failed transactions are diagnostic counters. Protection uses each motor's
    // last actual feedback time below, rather than failures across a leg sweep.
    // Skills and running may exceed the ordinary joint-speed limit during their
    // motion or lowering phase. Preserve feedback freshness, finite-value,
    // driver-error and temperature checks throughout these modes.
    const bool model_velocity_motion = mode_ == Mode::kWave || mode_ == Mode::kForwardJump ||
        mode_ == Mode::kSit ||
        mode_ == Mode::kRightFlip || mode_ == Mode::kRun;
    bool stale = false, hot = false, position_out_of_range = false;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const auto &timed = s.motors.joints[i];
        const auto &f = timed.feedback;
        const double age = seconds(now - timed.received_at);
        // v7 asymmetric thigh limits; hips/calves use the configured absolute limit.
        const float low = i % 3 == 1 ? (i < 6 ? -kTwoPi / 4.F : -kTwoPi / 12.F) : -config_.maximum_position;
        const float high = i % 3 == 1
            ? (i < 6 ? kTwoPi * 5.F / 9.F : kTwoPi * 23.F / 36.F)
            : config_.maximum_position;
        if (!timed.valid) {
            reason = "motor " + std::to_string(i) + " feedback invalid";
            return Health::kUnsafe;
        }
        if (!f.position_anchored) {
            reason = "motor " + std::to_string(i) + " session anchor invalid";
            return Health::kUnsafe;
        }
        if (!std::isfinite(f.position_rad) || !std::isfinite(f.velocity_rad_s)) {
            reason = "motor " + std::to_string(i) + " feedback non-finite";
            return Health::kUnsafe;
        }
        if (f.error != 0) {
            reason = "motor " + std::to_string(i) + " driver error=" + std::to_string(static_cast<unsigned>(f.error));
            return Health::kUnsafe;
        }
        // On new_bigcore, every UART shares the same native-UART transaction path.
        // A timed-out transaction keeps that motor's last verified sample, so use
        // the same bounded freshness grace for all twelve motors instead of only
        // the historically degraded RR UART6 leg.
        const bool degraded_uart = config_.rr_uart6_degraded;
        const double soft_timeout = degraded_uart ? config_.rr_motor_soft_timeout : config_.motor_soft_timeout;
        const double hard_timeout = degraded_uart ? config_.rr_motor_hard_timeout : config_.motor_hard_timeout;
        if (age < 0 || age > hard_timeout) {
            reason = "motor " + std::to_string(i) + " feedback stale age=" + std::to_string(age);
            return Health::kUnsafe;
        }
        if (!model_velocity_motion && std::fabs(f.velocity_rad_s) > config_.maximum_velocity) {
            reason = "motor " + std::to_string(i) + " velocity=" + std::to_string(f.velocity_rad_s);
            return Health::kUnsafe;
        }
        position_out_of_range = position_out_of_range ||
            (enforce_position_limits && (f.position_rad < low || f.position_rad > high));
        // The motor worker keeps the last verified sample when a new_bigcore UART
        // transaction times out.  Do not turn that expected read failure into a
        // controller stale state or protective crouch during the bounded grace
        // period.  All twelve motors use the same rule.
        stale = stale || age > soft_timeout;
        hot = hot || f.temperature_c > config_.maximum_temperature;
    }
    // A position-limit violation is recoverable only when every other motor
    // safety check is still good.  Keep mixed position+stale/hot feedback on
    // the fail-closed path rather than attempting a coordinated crouch from
    // incomplete or thermally unsafe state.
    if (position_out_of_range) {
        if (stale || hot) {
            reason = "motor position out of range with stale or hot feedback";
            return Health::kUnsafe;
        }
        reason = "motor position outside configured limit; protective crouch required";
        return Health::kPositionOutOfRange;
    }
    if (stale) {
        reason = "transient motor feedback stale";
        return Health::kStale;
    }
    if (hot) {
        reason = "motor exceeds temperature limit";
        return Health::kTooHot;
    }
    reason.clear();
    return Health::kFresh;
}

bool Controller::imu_fresh(const ImuSample &imu, TimePoint now) const {
    if (!imu.valid)
        return false;
    if (now < imu.received_at || seconds(now - imu.received_at) > config_.imu_soft_timeout) {
        return false;
    }
    for (float v : imu.angular_velocity)
        if (!std::isfinite(v))
            return false;
    float norm = 0;
    for (float v : imu.quaternion)
        norm += v * v;
    return std::isfinite(norm) && norm > 1e-6F;
}
void Controller::invalidate_policy() {
    ++generation_;
    target_valid_ = false;
    target_variant_ = 0;
    inference_fault_.clear();
}
void Controller::reset_skill_progress() {
    policy_frame_ = 0U;
    wave_tick_ = 0U;
    wave_raised_ = true;
    rollover_step_ = 0;
    rollover_step_running_ = false;
}

void Controller::passive(const std::string &reason) {
    passive_after_exit_ = false;
    exit_gait_until_.reset();
    sit_exit_start_.reset();
    sit_exit_stable_since_.reset();
    terrain_to_walk_pending_ = false;
    flip_walk_pending_ = false;
    mode_ = Mode::kPassive;
    pending_mode_.reset();
    protective_ = false;
    passive_crouch_ = false;
    last_valid_ = stale_hold_ = reseed_ = false;
    pose_a_recovery_ = false;
    velocity_ = {};
    ramped_velocity_ = {};
    velocity_at_ = {};
    ramped_velocity_at_ = {};
    invalidate_policy();
    cts_history_.fill(0.F);
    cts_history_initialized_ = false;

    reset_skill_progress();
    reason_ = reason;
}
void Controller::crouch(const RobotState &s, TimePoint now, std::string reason) {
    passive_after_exit_ = false;
    enter(Mode::kStand, s, now);
    // Missing feedback does not prove that transmission has failed. Seed the
    // bounded stop from a finite anchored sample, or the last commanded pose.
    initial_ = kStandPose;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        if (last_valid_ && std::isfinite(last_[i].position_rad))
            initial_[i] = last_[i].position_rad;
        const auto &joint = s.motors.joints[i];
        if (joint.valid && joint.feedback.position_anchored && std::isfinite(joint.feedback.position_rad))
            initial_[i] = joint.feedback.position_rad;
    }
    protective_ = true;
    ++protective_crouch_id_;
    passive_crouch_ = reason == "user requested passive";
    reason_ = reason;
    velocity_ = {};
    velocity_at_ = {};
}
void Controller::enter(Mode mode, const RobotState &s, TimePoint now) {
    exit_gait_until_.reset();
    sit_exit_start_.reset();
    sit_exit_stable_since_.reset();
    terrain_to_walk_pending_ = false;
    flip_walk_pending_ = false;
    mode_ = mode;
    initial_ = positions(s);
    transition_at_ = policy_at_ = now;
    protective_ = stale_hold_ = reseed_ = false;
    passive_crouch_ = false;
    pose_a_recovery_ = false;
    ramped_velocity_ = {};
    ramped_velocity_at_ = {};
    invalidate_policy();
    // Every policy starts with a clean action/history state.  This matters for
    // finite motion policies whose `time_step` is owned by the lazy Policy
    // instance and for history-stacked running policies.
    if (mode != Mode::kPassive && mode != Mode::kCrouch && mode != Mode::kStand) {
        previous_.fill(0);
        previous_previous_.fill(0);
    }
    cts_history_.fill(0.0F);

    cts_history_initialized_ = false;

    reset_skill_progress();
    if (mode == Mode::kWave || mode == Mode::kSit || mode == Mode::kForwardJump) {
        target_ = config_.policy_adapter->action_to_hardware_targets({}, mode, 0.F, config_.sit_reference);
        target_valid_ = true;
    }
    reason_.clear();
}
void Controller::finish_skill(const RobotState &s, TimePoint now) {
    const Mode next = pending_mode_.value_or(Mode::kStand);
    pending_mode_.reset();
    velocity_ = {};
    velocity_at_ = {};
    if (requires_fixstand(next)) {
        enter(Mode::kStand, s, now);
        pending_mode_ = next;
        reason_ = "skill recovery complete; FixStand before " + std::string(mode_name(next));
    } else {
        enter(next, s, now);
        reason_ = "skill recovery complete";
    }
}

Commands Controller::remember(Commands commands) {
    last_ = commands;
    last_valid_ =
        std::any_of(commands.begin(), commands.end(), [](const auto &c) { return c.mode == MotorMode::kFoc; });
    return commands;
}

void Controller::set_velocity(VelocityCommand command, TimePoint now) {
    if (!std::isfinite(command.vx) || !std::isfinite(command.vy) || !std::isfinite(command.yaw)) {
        throw std::invalid_argument("velocity command contains NaN or infinity");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    velocity_ = command;
    velocity_at_ = now;
}

VelocityCommand Controller::ramp_velocity(VelocityCommand target, TimePoint now) {
    // The source command can arrive at either 20 Hz (web) or 50 Hz (SBUS), so
    // slew against elapsed steady-clock time rather than assuming a producer
    // period.  A bounded dt prevents a delayed callback from creating a jump.
    const auto &spec = config_.policy_adapter->spec_for(mode_);
    const VelocityCommand bounded_target{
        std::clamp(target.vx, spec.vx_lo, spec.vx_hi), std::clamp(target.vy, spec.vy_lo, spec.vy_hi),
        std::clamp(target.yaw, spec.yaw_lo, spec.yaw_hi)};
    const bool running = mode_ == Mode::kRun;
    const double linear_acceleration = running ? config_.run_linear_acceleration : config_.move_linear_acceleration;
    const double linear_deceleration = running ? config_.run_linear_deceleration : config_.move_linear_deceleration;
    const double yaw_acceleration = running ? config_.run_yaw_acceleration : config_.move_yaw_acceleration;
    const double yaw_deceleration = running ? config_.run_yaw_deceleration : config_.move_yaw_deceleration;
    if (ramped_velocity_at_ == TimePoint{} || now < ramped_velocity_at_) {
        ramped_velocity_ = {};
        ramped_velocity_at_ = now;
        return ramped_velocity_;
    }
    const double dt = std::clamp(seconds(now - ramped_velocity_at_), 0.0, 0.10);
    ramped_velocity_at_ = now;
    ramped_velocity_.vx = slew(ramped_velocity_.vx, bounded_target.vx, linear_acceleration * dt,
        linear_deceleration * dt);
    ramped_velocity_.vy = slew(ramped_velocity_.vy, bounded_target.vy, linear_acceleration * dt,
        linear_deceleration * dt);
    ramped_velocity_.yaw = slew(
        ramped_velocity_.yaw, bounded_target.yaw, yaw_acceleration * dt, yaw_deceleration * dt);
    return ramped_velocity_;
}

ModeResult Controller::set_mode(Mode next, const RobotState &s, TimePoint now, bool finish_motion) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string reason;
    const auto health = motor_health(s, now, reason);
    const bool requesting_passive = next == Mode::kPassive;
    if (next == Mode::kPassive) {
        if (mode_ == Mode::kPassive)
            return {true, "already passive"};
        if (protective_)
            return {true, "crouch already in progress; BRAKE follows"};
        const bool flip_recovery = mode_ == Mode::kRightFlip;
        const bool has_exit = mode_ == Mode::kWave ||
            mode_ == Mode::kSit || mode_ == Mode::kForwardJump || flip_recovery ||
            mode_ == Mode::kRollover || exit_gait_until_ || flip_walk_pending_;
        if (finish_motion && health == Health::kFresh && imu_fresh(s.imu, now) && !imu_latched_ &&
            (has_exit || passive_after_exit_)) {
            if (passive_after_exit_)
                return {true, "motion exit before crouch and passive already requested"};
            passive_after_exit_ = true;
            velocity_ = {};
            velocity_at_ = {};
            if (flip_recovery || mode_ == Mode::kRollover || exit_gait_until_ ||
                flip_walk_pending_) {
                reason_ = "finishing recovery before crouch and passive";
                return {true, reason_};
            }
            next = Mode::kStand;
        } else {
            pending_mode_.reset();
            crouch(s, now, "user requested passive" + (reason.empty() ? std::string{} : "; " + reason));
            return {true, "passive requested"};
        }
    }
    if (protective_)
        return {false, "protective crouch is in progress"};
    // An explicit pose request cancels a previously queued model transition.
    if (next == Mode::kCrouch || next == Mode::kStand)
        pending_mode_.reset();
    bool pose_a_recovery = false;
    if (health != Health::kFresh) {
        // Match v9's narrowly scoped recovery path: Passive -> Sit may ignore
        // feedback position limits only. Every other motor safety check remains
        // mandatory, and no other active mode can use this exception.
        if (mode_ != Mode::kPassive || next != Mode::kCrouch || motor_health(s, now, reason, false) != Health::kFresh) {
            return {false, reason};
        }
        pose_a_recovery = true;
    }
    if (!imu_fresh(s.imu, now))
        return {false, "fresh body IMU required"};
    if (imu_latched_)
        return {false, "wait for safety loop to acknowledge IMU recovery"};
    if (next == Mode::kRollover && !rollover_tilted(s.imu)) {
        return {false, "rollover requires |roll| > 60 deg or |pitch| > 60 deg"};
    }
    if (!requesting_passive)
        passive_after_exit_ = false;
    if (mode_ == Mode::kSit && (next != mode_ || sit_exit_start_)) {
        pending_mode_ = next;
        if (!sit_exit_start_) {
            sit_exit_start_ = std::max(policy_frame_ * kSkillControlDt, kSitEndSeconds);
            sit_exit_stable_since_.reset();
            ++generation_; // Reject inference sampled before the stand request.
        }
        reason_ = "sit recovery before " + std::string(mode_name(next));
        return {true, reason_};
    }
    if (mode_ == Mode::kForwardJump && next != mode_) {
        pending_mode_ = next;
        reason_ = "finishing motion before " + std::string(mode_name(next));
        return {true, reason_};
    }
    if (mode_ == Mode::kWave && (next == Mode::kStand || passive_after_exit_)) {
        if (wave_raised_)
            ++generation_;
        wave_raised_ = false;
        reason_ = "lowering wave paw before recovery";
        return {true, reason_};
    }

    if (next == mode_) {
        // An explicit same-mode selection cancels the automatic exit sequence.

        exit_gait_until_.reset();
        terrain_to_walk_pending_ = false;
        flip_walk_pending_ = false;
        if (next == Mode::kWave && !wave_raised_) {
            ++generation_;
            wave_raised_ = true;
            reason_.clear();
        }
        return {true, "already in requested mode"};
    }

    if (requires_fixstand(next)) {
        const bool fixstand_ready = next != Mode::kWave && next != Mode::kSit && next != Mode::kForwardJump &&
            mode_ == Mode::kStand && seconds(now - transition_at_) >= config_.stand_duration &&
            last_valid_;
        if (!fixstand_ready) {
            // Queue the requested policy while ensuring the robot first reaches the
            // fixed standing pose. Wave needs its own nominal stance, so it always
            // starts a fresh transition; other policies reuse an ongoing FixStand.
            pending_mode_ = next;
            if (mode_ != Mode::kStand || next == Mode::kWave || next == Mode::kSit || next == Mode::kForwardJump)
                enter(Mode::kStand, s, now);
            reason_ = "FixStand before " + std::string(mode_name(next));
            return {
                true, "FixStand started; target mode " + std::string(mode_name(next)) + " will start after transition"};
        }
    }
    pending_mode_.reset();
    enter(next, s, now);
    pose_a_recovery_ = pose_a_recovery;
    return {
        true, pose_a_recovery ? "crouch out-of-range recovery started" : std::string("mode set to ") + mode_name(next)};
}

ModeResult Controller::step_rollover(const RobotState &s, TimePoint now) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string reason;
    if (protective_)
        return {false, "protective crouch is in progress"};
    if (motor_health(s, now, reason) != Health::kFresh)
        return {false, reason};
    if (!imu_fresh(s.imu, now) || imu_latched_)
        return {false, "fresh body IMU required"};
    if (mode_ != Mode::kRollover) {
        if (mode_ != Mode::kPassive && mode_ != Mode::kCrouch)
            return {false, "rollover step test requires passive or crouch"};
        if (!rollover_tilted(s.imu))
            return {false, "rollover requires |roll| > 60 deg or |pitch| > 60 deg"};
        pending_mode_.reset();
        passive_after_exit_ = false;
        enter(Mode::kRollover, s, now);
    } else if (rollover_step_ == 0) {
        return {false, "automatic rollover is running"};
    } else if (seconds(now - rollover_step_at_) < kRolloverSegmentDuration) {
        return {false, "current rollover step is still running"};
    }
    if (rollover_step_ >= kRolloverSegmentCount)
        return {false, "all rollover steps complete; holding Pose A"};
    ++rollover_step_;
    rollover_step_at_ = now;
    rollover_step_running_ = true;
    reason_ = "rollover step " + std::to_string(rollover_step_) + "/" +
        std::to_string(kRolloverSegmentCount) + " running";
    return {true, reason_};
}

Commands Controller::tick(const RobotState &s, TimePoint now) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto apply_flip_pacing = [this](Commands commands) {
        const bool paced = !protective_ && mode_ == Mode::kRightFlip;
        for (auto &command : commands)
            command.flip_uart_pacing = paced;
        return commands;
    };
    // Preserve priority: motor health, completed stand transition, IMU/inference
    // watchdogs, recovery transitions, then the current mode's command.
    if (auto commands = check_motor_state(s, now))
        return apply_flip_pacing(*commands);
    if (auto commands = advance_stand_transition(s, now))
        return apply_flip_pacing(*commands);
    if (auto commands = check_policy_health(s, now))
        return apply_flip_pacing(*commands);
    advance_recovery(s, now);
    if (passive_after_exit_ && !protective_ &&
        ((mode_ == Mode::kStand && !pending_mode_) ||
            (mode_ == Mode::kRollover && (rollover_step_ > 0 ||
                seconds(now - transition_at_) >= kRolloverSegmentCount * kRolloverSegmentDuration)))) {
        crouch(s, now, "user requested passive");
    }
    return apply_flip_pacing(mode_commands(s, now));
}

std::optional<Commands> Controller::check_motor_state(const RobotState &s, TimePoint now) {
    std::string reason;
    auto health = motor_health(s, now, reason);
    if (pose_a_recovery_ && mode_ == Mode::kCrouch && health == Health::kUnsafe)
        health = motor_health(s, now, reason, false);

    // Startup is passive by design.  The motor workers may still be building
    // their session anchors, so an invalid/stale snapshot here means
    // "waiting for feedback", not a runtime fault.  Never send a crouch FOC
    // command before the controller has received a complete valid snapshot and
    // entered active control at least once.
    if (!protective_ && mode_ == Mode::kPassive && !last_valid_) {
        if (imu_fresh(s.imu, now)) {
            imu_latched_ = false;
            imu_stale_since_.reset();
        }
        if (reason_.empty())
            reason_ = "waiting for valid motor feedback";
        return remember({});
    }

    // Once protection has started, keep sending the coordinated crouch even if
    // the same transport fault remains visible.  Clearing FOC here would drop
    // the robot before it reaches the safe crouched pose.
    if (protective_)
        return mode_commands(s, now);

    // Motor faults are checked even when IMU is stale.  Start the protective
    // crouch first; mode_commands() will switch to passive only after the
    // configured crouch duration has elapsed.
    if (health == Health::kUnsafe) {
        crouch(s, now, reason);
        return mode_commands(s, now);
    }
    if (pose_a_recovery_ && health == Health::kTooHot) {
        crouch(s, now, reason);
        return mode_commands(s, now);
    }
    if (mode_ == Mode::kPassive) {
        if (imu_fresh(s.imu, now)) {
            imu_latched_ = false;
            imu_stale_since_.reset();
        }
        return remember({});
    }
    if (health == Health::kStale) {
        if (!last_valid_) {
            crouch(s, now, reason);
            return mode_commands(s, now);
        }
        if (!stale_hold_) {
            invalidate_policy();
            if (mode_ == Mode::kWalk)
                mode_ = Mode::kStand;
            stale_hold_ = reseed_ = true;
            velocity_ = {};
            velocity_at_ = {};
        }
        reason_ = reason;
        return last_;
    }
    if (reseed_) {
        initial_ = positions(s);
        transition_at_ = now;
        reseed_ = stale_hold_ = false;
    }
    if (pose_a_recovery_ && motor_health(s, now, reason) == Health::kFresh) {
        pose_a_recovery_ = false;
        reason_.clear();
    }
    if (health == Health::kPositionOutOfRange && !protective_ && !pose_a_recovery_) {
        crouch(s, now, reason);
    }
    if (health == Health::kTooHot && !protective_)
        crouch(s, now, reason);

    return std::nullopt;
}

std::optional<Commands> Controller::advance_stand_transition(const RobotState &s, TimePoint now) {
    if (!protective_ && pending_mode_ && mode_ == Mode::kStand &&
        seconds(now - transition_at_) >= config_.stand_duration && last_valid_) {
        const Mode next = *pending_mode_;
        const bool timed_flip_walk = flip_walk_pending_ && next == Mode::kWalk;
        pending_mode_.reset();
        // Emit the completed FixStand target once before the first policy frame;
        // otherwise the new mode would reuse the previous interpolated command
        // for one control tick and visibly stop short of the fixed pose.
        const Joints final_fixstand_target = next == Mode::kRightFlip
            ? kStandPose
            : action_entry_target(*config_.policy_adapter, next, config_.sit_reference);
        const auto final_fixstand = pose_commands(*config_.policy_adapter, final_fixstand_target, false);
        enter(next, s, now);
        if (timed_flip_walk) {
            exit_gait_until_ = now + std::chrono::seconds(2);
            reason_ = "flip recovery; Walk for 2 s before FixStand";
        }
        return remember(final_fixstand);
    }

    if (!protective_ && mode_ == Mode::kWave && !wave_raised_ && wave_tick_ == 0U) {
        enter(Mode::kStand, s, now);
    }

    return std::nullopt;
}

std::optional<Commands> Controller::check_policy_health(const RobotState &s, TimePoint now) {
    if (imu_fresh(s.imu, now)) {
        imu_stale_since_.reset();
        if (!protective_)
            imu_latched_ = false;
        check_walking_tilt(s, now);
    } else if (!protective_) {
        if (!imu_stale_since_) {
            const auto soft_start = now -
                std::chrono::duration_cast<SteadyClock::duration>(
                    std::chrono::duration<double>(config_.imu_soft_timeout));
            imu_stale_since_ = s.imu.received_at == TimePoint{} ? soft_start : std::min(s.imu.received_at, soft_start);
            invalidate_policy();
        }
        if (seconds(now - *imu_stale_since_) <= config_.imu_hard_timeout) {
            reason_ = "body IMU stale; holding last verified command";
            return last_valid_ ? last_ : Commands{};
        }
        imu_latched_ = true;
        if (aerial_flip(mode_)) {
            reason_ = "body IMU hard timeout; holding flip command";
            return last_valid_ ? last_ : Commands{};
        }
        crouch(s, now, "body IMU hard timeout");
    }

    if (!protective_ && mode_ != Mode::kPassive && mode_ != Mode::kCrouch && mode_ != Mode::kStand &&
        mode_ != Mode::kRollover) {
        if (!inference_fault_.empty()) {
            if (aerial_flip(mode_)) {
                reason_ = "flip inference fault; holding last command: " + inference_fault_;
                return last_valid_ ? last_ : Commands{};
            }
            crouch(s, now, inference_fault_);
        } else if (seconds(now - policy_at_) > config_.motor_hard_timeout) {
            if (aerial_flip(mode_)) {
                reason_ = "no timely flip policy result; holding last command";
                return last_valid_ ? last_ : Commands{};
            }
            crouch(s, now, "no timely policy result; hard timeout");
        }
    }
    return std::nullopt;
}

void Controller::check_walking_tilt(const RobotState &s, TimePoint now) {
    const bool walking = mode_ == Mode::kWalk || mode_ == Mode::kRun || mode_ == Mode::kTerrain;
    if (config_.walking_tilt_protection && !protective_ && walking && imu_fresh(s.imu, now) &&
        tilt_exceeds(s.imu, rollover_rad(40.F), rollover_rad(60.F))) {
        crouch(s, now, "walking tilt exceeded |roll| 40 deg or |pitch| 60 deg");
    }
}

void Controller::advance_recovery(const RobotState &s, TimePoint now) {
    if (!protective_ && mode_ == Mode::kSit && sit_exit_start_) {
        const float elapsed = policy_frame_ * kSkillControlDt - *sit_exit_start_;
        const auto &q = s.imu.six_axis_quaternion;
        float norm = 0.F;
        for (float v : q)
            norm += v * v;
        const float upright_z = norm > 1e-6F ? 1.F - 2.F * (q[1] * q[1] + q[2] * q[2]) / norm : -1.F;
        bool settled = std::isfinite(upright_z) && upright_z > std::cos(.10F);
        float angular_speed_sq = 0.F;
        for (float v : s.imu.angular_velocity)
            angular_speed_sq += v * v;
        settled = settled && angular_speed_sq < .25F;
        for (const auto &j : s.motors.joints)
            settled = settled && std::fabs(j.feedback.velocity_rad_s) < .8F;
        if (elapsed >= kSitRecoverySeconds && settled) {
            if (!sit_exit_stable_since_)
                sit_exit_stable_since_ = now;
            if (seconds(now - *sit_exit_stable_since_) >= .3) {
                finish_skill(s, now);
            }
        } else
            sit_exit_stable_since_.reset();
        if (mode_ == Mode::kSit && elapsed >= kSitRecoveryTimeoutSeconds) {
            pending_mode_.reset();
            crouch(s, now, "sit recovery did not settle; target mode cancelled");
        }
    }

    if (exit_gait_until_ && !protective_ && (mode_ == Mode::kTerrain || mode_ == Mode::kWalk) &&
        now >= *exit_gait_until_) {
        pending_mode_.reset();
        const bool start_walk = mode_ == Mode::kTerrain && terrain_to_walk_pending_;
        enter(start_walk ? Mode::kWalk : Mode::kStand, s, now);
        velocity_ = {};
        velocity_at_ = {};
        if (start_walk) {
            exit_gait_until_ = now + std::chrono::milliseconds(1200);
            reason_ = "upright exit; Walk for 1.2 s before FixStand";
        } else {
            reason_ = "exit gait complete; FixStand";
        }
    }
}

Commands Controller::mode_commands(const RobotState &s, TimePoint now) {
    if (protective_) {
        double elapsed = seconds(now - transition_at_);
        bool final_sent = true;
        if (s.motors.transmissions_tracked) {
            elapsed = std::numeric_limits<double>::infinity();
            for (const auto &sent : s.motors.transmissions) {
                const bool current = sent.protective_crouch_id == protective_crouch_id_;
                elapsed = std::min(elapsed, current ? sent.protective_elapsed_sec : 0.0);
                final_sent = final_sent && current && sent.protective_active && sent.protective_final;
            }
        }
        if (elapsed >= config_.crouch_duration && final_sent) {
            passive("protective crouch complete; cause: " + reason_);
            return remember({});
        }
        auto commands = pose_commands(*config_.policy_adapter,
            interpolate(initial_, config_.crouch_pose, elapsed, config_.crouch_duration), false);
        for (auto &command : commands) {
            command.protective_crouch = true;
            command.protective_crouch_id = protective_crouch_id_;
            command.protective_crouch_final = elapsed >= config_.crouch_duration;
        }
        return remember(commands);
    }

    if (mode_ == Mode::kStand || mode_ == Mode::kCrouch) {
        const Joints pose = mode_ == Mode::kStand &&
                (pending_mode_ == Mode::kWave || pending_mode_ == Mode::kSit ||
                    pending_mode_ == Mode::kForwardJump)
            ? action_entry_target(*config_.policy_adapter, *pending_mode_, config_.sit_reference)
            : (mode_ == Mode::kStand ? kStandPose : config_.crouch_pose);
        return remember(pose_commands(*config_.policy_adapter,
            interpolate(initial_, pose, seconds(now - transition_at_),
                mode_ == Mode::kStand ? config_.stand_duration : config_.crouch_pose_duration),
            false));
    }
    if (mode_ == Mode::kRollover) {
        // Keep the action in the active mode until its final Pose A segment has
        // completed; the next web command (stand/passive/walk) can then take over.
        double elapsed = seconds(now - transition_at_);
        if (rollover_step_ > 0) {
            const double step_elapsed = std::clamp(seconds(now - rollover_step_at_), 0.0, kRolloverSegmentDuration);
            elapsed = (rollover_step_ - 1) * kRolloverSegmentDuration + step_elapsed;
            if (rollover_step_running_ && step_elapsed >= kRolloverSegmentDuration) {
                rollover_step_running_ = false;
                reason_ = "rollover step " + std::to_string(rollover_step_) + "/" +
                    std::to_string(kRolloverSegmentCount) + " holding";
            }
        }
        return remember(pose_commands(*config_.policy_adapter,
            rollover_target(initial_, config_.crouch_pose, elapsed), false));
    }
    if (mode_ != Mode::kPassive && mode_ != Mode::kCrouch && mode_ != Mode::kStand && target_valid_) {
        Joints commanded = target_;

        if (mode_ == Mode::kWave || mode_ == Mode::kSit) {
            Commands commands{};
            const auto torques = config_.policy_adapter->wave_torques(commanded, s, mode_);
            for (std::size_t i = 0; i < kNumJoints; ++i) {
                commands[i].mode = MotorMode::kFoc;
                commands[i].torque_nm = torques[i];
            }
            return remember(commands);
        }
        Commands commands = pose_commands(*config_.policy_adapter, commanded, true, mode_, target_variant_);
        if (aerial_flip(mode_)) {
            // Keep the aerial motion tag while the jump uses position control.
            for (auto &command : commands)
                command.backflip_control = true;
        }
        return remember(commands);
    }
    return last_valid_ ? last_ : Commands{};
}

std::optional<PolicyRequest> Controller::prepare_policy(const RobotState &s, TimePoint now) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string reason;
    if (mode_ == Mode::kPassive || mode_ == Mode::kCrouch || mode_ == Mode::kStand || mode_ == Mode::kRollover ||
        protective_ || !imu_fresh(s.imu, now) || motor_health(s, now, reason) != Health::kFresh)
        return std::nullopt;
    check_walking_tilt(s, now);
    if (protective_)
        return std::nullopt;
    const auto command = policy_command(now);
    if (!policy_phase_ready(s, now))
        return std::nullopt;
    return prepare_mode_observation(s, command, now);
}

VelocityCommand Controller::policy_command(TimePoint now) {
    if (passive_after_exit_)
        return {};
    const bool command_fresh =
        velocity_at_ != TimePoint{} && now >= velocity_at_ && seconds(now - velocity_at_) <= config_.command_timeout;
    if (command_fresh && (mode_ == Mode::kRun || mode_ == Mode::kWalk || mode_ == Mode::kTerrain))
        return ramp_velocity(velocity_, now);
    // A stale producer drops the input to zero and restarts its ramp after reconnecting.
    ramped_velocity_ = {};
    ramped_velocity_at_ = {};
    return command_fresh ? velocity_ : VelocityCommand{};
}

void Controller::begin_flip_recovery(const RobotState &s, TimePoint now) {
    enter(Mode::kStand, s, now);
    velocity_ = {};
    velocity_at_ = {};
    pending_mode_ = Mode::kWalk;
    flip_walk_pending_ = true;
    reason_ = "flip complete; FixStand before Walk";
}

bool Controller::policy_phase_ready(const RobotState &s, TimePoint now) {
    if (mode_ == Mode::kForwardJump && policy_frame_ >= kForwardJumpFrames) {
        finish_skill(s, now);
        return false;
    }
    // The ONNX reference has frames 0..118. Finish only after all actions
    // were accepted; prepare_policy runs on the following policy interval.
    if (mode_ == Mode::kRightFlip && policy_frame_ >= kRightFlipReferenceFrames) {
        begin_flip_recovery(s, now);
        return false;
    }

    return true;
}

PolicyRequest Controller::prepare_mode_observation(const RobotState &s, VelocityCommand command, TimePoint now) {
    const float reference_time =
        mode_ == Mode::kWave ? next_wave_tick(wave_tick_, wave_raised_) * kWaveControlDt : 0.0F;
    std::vector<float> observation;
    if (mode_ == Mode::kSit) {
        observation = config_.policy_adapter->build_sit_observation(config_.sit_reference, s, previous_,
            (policy_frame_ + 1U) * kSkillControlDt, sit_exit_start_.value_or(std::numeric_limits<float>::infinity()));
    } else {
        observation = config_.policy_adapter->build_observation(s, command, previous_, mode_, reference_time);
    }
    if (mode_ == Mode::kRun || mode_ == Mode::kTerrain) {
        update_history<5>(cts_history_, cts_history_initialized_, observation, HistoryOrder::kOldestFirst);
        const auto dims = config_.policy_adapter->observation_term_dimensions(mode_);
        std::vector<float> term_major;
        term_major.reserve(225);
        std::size_t offset = 0;
        for (const auto dimension : dims) {
            for (std::size_t frame_index = 0; frame_index < 5; ++frame_index) {
                term_major.insert(term_major.end(), cts_history_.begin() + frame_index * 45 + offset,
                    cts_history_.begin() + frame_index * 45 + offset + dimension);
            }
            offset += dimension;
        }
        observation = std::move(term_major);
    }
    const auto request_frame = policy_frame_;
    // Finite/reference motions advance only when an action is accepted.
    if (mode_ != Mode::kWave && mode_ != Mode::kRightFlip &&
        mode_ != Mode::kSit && mode_ != Mode::kForwardJump) {
        ++policy_frame_;
    }
    return PolicyRequest{mode_, std::move(observation), s, request_frame, generation_, now};
}
void Controller::complete_policy(const PolicyRequest &request, const Action &action, TimePoint now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ != request.mode || protective_ || request.generation != generation_)
        return;
    try {
        if (now < request.sampled_at || seconds(now - request.sampled_at) > config_.motor_soft_timeout) {
            throw std::runtime_error("policy result arrived too late");
        }
        Action accepted_action = action;
        const std::size_t variant_index = 0;
        const auto &spec = config_.policy_adapter->variant(request.mode, variant_index);
        for (auto &value : accepted_action) {
            if (!std::isfinite(value))
                throw std::invalid_argument("non-finite policy action");
            const float clip = spec.last_action_clip > 0    ? spec.last_action_clip
                : spec.loaded.exec_cfg.clip_actions.empty() ? 0.F
                                                            : spec.loaded.exec_cfg.clip_actions.front();
            if (clip > 0)
                value = std::clamp(value, -clip, clip);
        }
        Joints target{};
        {
            const float reference_time =
                request.mode == Mode::kWave ? next_wave_tick(wave_tick_, wave_raised_) * kWaveControlDt : 0.0F;
            target = request.mode == Mode::kSit
                ? config_.policy_adapter->sit_hardware_targets(config_.sit_reference, accepted_action,
                        (request.policy_frame + 1U) * kSkillControlDt,
                        sit_exit_start_.value_or(std::numeric_limits<float>::infinity()))
                : config_.policy_adapter->action_to_hardware_targets(
                        accepted_action, request.mode, reference_time, {}, variant_index);
        }
        target_ = target;
        target_variant_ = variant_index;
        previous_previous_ = previous_;
        previous_ = accepted_action;
        target_valid_ = true;
        policy_at_ = now;

        if (request.mode == Mode::kWave) {
            wave_tick_ = next_wave_tick(wave_tick_, wave_raised_);
        }
        if (request.mode == Mode::kWave || request.mode == Mode::kSit || request.mode == Mode::kForwardJump ||
            request.mode == Mode::kRightFlip) {
            policy_frame_ = std::max(policy_frame_, request.policy_frame + 1U);
        }
    } catch (const std::exception &error) {
        inference_fault_ = error.what();
        target_valid_ = false;
    }
}
void Controller::fail_policy(const PolicyRequest &request, const std::string &reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == request.mode && request.generation == generation_) {
        inference_fault_ = reason;
        target_valid_ = false;
    }
}

ControllerStatus Controller::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {mode_, protective_, reason_, rollover_step_, rollover_step_running_, kRolloverSegmentCount};
}
} // namespace go2
