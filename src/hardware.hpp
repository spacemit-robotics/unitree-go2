/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef HARDWARE_HPP
#define HARDWARE_HPP
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace go2 {

constexpr std::size_t kNumGo2Legs = 4;
constexpr std::size_t kMotorsPerLeg = 3;
constexpr std::size_t kNumGo2Motors = kNumGo2Legs * kMotorsPerLeg;
constexpr float kGo2StandardGearRatio = 6.33F;
constexpr float kGo2CalfJointGearRatio = 12.13F;
constexpr float kTwoPi = 6.28318530717958647692F;
// The Go2 internal motor bus is fixed at 4 Mbaud and is not runtime-configurable.
constexpr std::uint32_t kMotorBaudrate = 4000000U;

enum class MotorMode : std::uint8_t {
    kBrake = 0,
    kFoc = 1,
};

struct JointCalibration {
    std::uint8_t motor_id{0};
    float gear_ratio{kGo2StandardGearRatio};
    float direction{1.0F};
    float joint_zero_offset_rad{0.0F};
};

struct JointCommand {
    MotorMode mode{MotorMode::kBrake};
    float torque_nm{0.0F};
    float velocity_rad_s{0.0F};
    float position_rad{0.0F};
    float kp{0.0F};
    float kd{0.0F};
    // The flip actors own a complete aerial launch/landing episode.  The direct
    // serial backend uses this tag to avoid turning a transient reply loss into
    // the v9 process-wide session-anchor BRAKE latch.  Set for every flip episode
    // (right/left/forward/back).
    bool backflip_control{false};
    // A bounded controller stop may transmit using this session's last known
    // reference despite missing feedback. Ordinary FOC cannot use this tag.
    bool protective_crouch{false};
    std::uint64_t protective_crouch_id{0};
    bool protective_crouch_final{false};
    // Native UART workers use a 5 ms sweep only during front/back/side flips.
    bool flip_uart_pacing{false};
};

struct RotorCommand {
    std::uint8_t motor_id{0};
    MotorMode mode{MotorMode::kBrake};
    float torque_nm{0.0F};
    float velocity_rad_s{0.0F};
    float position_rad{0.0F};
    float kp{0.0F};
    float kd{0.0F};
};

struct RotorFeedback {
    std::uint8_t motor_id{0};
    float torque_nm{0.0F};
    float velocity_rad_s{0.0F};
    float position_rad{0.0F};
    int temperature_c{0};
    std::uint8_t error{0};
    std::uint16_t foot_force_raw{0};
    std::uint64_t received_us{0};
};

struct JointFeedback {
    float torque_nm{0.0F};
    float velocity_rad_s{0.0F};
    float position_rad{0.0F};
    float rotor_torque_nm{0.0F};
    float rotor_velocity_rad_s{0.0F};
    float rotor_position_rad{0.0F};
    int temperature_c{0};
    std::uint8_t error{0};
    std::uint16_t foot_force_raw{0};
    bool position_anchored{false};
};

struct TimedJointFeedback {
    JointFeedback feedback{};
    std::chrono::steady_clock::time_point received_at{};
    bool valid{false};
};

// Successful complete command writes, independent of reply freshness.
struct JointTransmission {
    std::chrono::steady_clock::time_point sent_at{};
    std::uint64_t protective_crouch_id{0};
    double protective_elapsed_sec{0};
    bool protective_final{false};
    bool protective_active{false};
};

struct LegTimingStats {
    std::uint64_t completed_sweeps{0};
    std::uint64_t period_samples{0};
    std::uint64_t transaction_failures{0};
    std::uint64_t reconnect_attempts{0};
    std::uint64_t consecutive_failures{0};
    double mean_period_ms{0.0};
    double minimum_period_ms{0.0};
    double maximum_period_ms{0.0};
    double standard_deviation_ms{0.0};
};

// Live new_bigcore progress is sampled without waiting for a UART transaction.
struct NativeUartDiagnostics {
    const char* stage{"inactive"};
    int motor_id{-1}, command_mode{-1}, last_result{0};
    double stage_age_ms{0}, last_transaction_ms{0};
    std::uint64_t transactions{0}, failures{0};
};

struct MotorSnapshot {
    std::array<TimedJointFeedback, kNumGo2Motors> joints{};
    std::array<std::string, kNumGo2Legs> leg_errors{};
    std::array<LegTimingStats, kNumGo2Legs> leg_timing{};
    std::array<NativeUartDiagnostics, kNumGo2Legs> native_uart{};
    std::array<JointTransmission, kNumGo2Motors> transmissions{};
    bool transmissions_tracked{false};
    bool session_anchor_faulted{false};
    bool startup_pose_invalid{false};
};

struct JointSessionReference {
    float joint_position_rad{0.0F};
    float rotor_position_rad{0.0F};
};

struct LegConfig {
    std::string device;
    std::string driver{"drv_485_m8010_a"};
    std::array<JointCalibration, kMotorsPerLeg> calibration{};
    std::array<JointSessionReference, kMotorsPerLeg> session_reference{};
    bool startup_pose_verified{true};
    std::array<float, kMotorsPerLeg> startup_joint_tolerances_rad{kTwoPi / 36.0F, kTwoPi / 36.0F, kTwoPi / 36.0F};
    unsigned int startup_attempts{5};
    std::chrono::milliseconds reconnect_delay{100};
};

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;
constexpr std::size_t kNumJoints = kNumGo2Motors;
using Joints = std::array<float, kNumJoints>;
using Commands = std::array<JointCommand, kNumJoints>;

struct ImuSample {
    std::array<float, 3> angular_velocity{}, linear_acceleration{};
    std::array<float, 4> quaternion{1, 0, 0, 0};
    std::array<float, 4> six_axis_quaternion{1, 0, 0, 0};
    TimePoint received_at{};
    std::uint64_t sequence{0};
    bool valid{false};
};
struct RobotState {
    MotorSnapshot motors;
    ImuSample imu;
};

constexpr Joints kStandPose{0, .8F, -1.5F, 0, .8F, -1.5F, 0, .8F, -1.5F, 0, .8F, -1.5F};
constexpr std::array<const char*, kNumJoints> kJointNames{"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
constexpr std::array<const char*, kNumGo2Legs> kGo2LegNames{"FR", "FL", "RR", "RL"};

inline double seconds(SteadyClock::duration duration) { return std::chrono::duration<double>(duration).count(); }

enum class Backend { kMock, kSerial };

// Board wiring and motor-control owner; model inference is independent.
enum class BoardProfile { kLegacyUsb, kNewSmallcore, kNewBigcore };

struct HardwareConfig {
    Backend backend{Backend::kMock};
    BoardProfile board_profile{BoardProfile::kLegacyUsb};
    bool motor_io_authorized{false}, actuation_enabled{false};
    bool hardware_configuration_verified{false}, motor_ids_verified{false};
    bool joint_calibration_verified{false}, suspended_test_verified{false};
    std::array<LegConfig, kNumGo2Legs> legs{};
    std::string imu_device;
    std::string imu_driver{"drv_uart_iwt603"};
    int imu_baud{921600};
    std::string imu_gpiochip;
    int imu_gpio_line{0};
    // Vectors: sensor -> body. Quaternion: body -> sensor, right-multiplied
    // onto sensor -> world attitude; these two rotations must be inverses.
    std::array<float, 9> imu_to_body_matrix{1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::array<float, 4> imu_to_body_quaternion{1, 0, 0, 0};
    float imu_roll_offset_rad{0}, imu_pitch_offset_rad{0};
};

class JointSessionAnchor {
public:
    void initialize(float current_rotor_position_rad, const JointSessionReference& reference,
        const JointCalibration& calibration, float maximum_joint_error_rad);
    void invalidate();
    bool valid() const { return valid_; }
    bool can_command(const JointCommand& command) const {
        return valid_ || (command.protective_crouch && reference_initialized_);
    }

    JointFeedback to_joint_feedback(
        const RotorFeedback& feedback, const JointCalibration& calibration, bool enforce_step_continuity = true,
        TimePoint sampled_at = {});
    RotorCommand to_rotor_command(const JointCommand& command, const JointCalibration& calibration) const;

private:
    bool valid_{false};
    bool reference_initialized_{false};
    JointSessionReference reference_{};
    float session_reference_rotor_position_rad_{0.0F};
    float last_rotor_position_rad_{0.0F};
    float last_rotor_velocity_rad_s_{0.0F};
    TimePoint last_sampled_at_{};
};

void validate_calibration(const JointCalibration&);
RotorCommand joint_to_rotor_command(const JointCommand&, const JointCalibration&);
JointFeedback rotor_to_joint_feedback(const RotorFeedback&, const JointCalibration&);
void validate_hardware_config(const HardwareConfig&);
std::array<float, 3> rotate_vector(const std::array<float, 9>&, const std::array<float, 3>&);
std::array<float, 4> multiply_quaternion(const std::array<float, 4>&, const std::array<float, 4>&);
std::array<float, 4> add_roll_pitch_offsets(const std::array<float, 4>&, float, float);
ImuSample imu_sample_to_body(const ImuSample&, const HardwareConfig&);

// No I/O occurs in the constructor. start() opens devices only for an approved
// serial configuration; mock never constructs a serial or IMU driver.
class Hardware final {
public:
    explicit Hardware(HardwareConfig config);
    ~Hardware();
    Hardware(const Hardware&) = delete;
    Hardware& operator=(const Hardware&) = delete;
    void start();
    void stop();
    void replace_calibration(const std::array<float, kNumGo2Motors>& offsets,
        const std::array<float, kNumGo2Motors>& startup_rotors);
    RobotState read(TimePoint now = SteadyClock::now()) const;
    ImuSample read_imu(TimePoint now = SteadyClock::now()) const;
    void write(const Commands&);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace go2

#endif  // HARDWARE_HPP
