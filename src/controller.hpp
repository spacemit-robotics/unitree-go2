#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include "hardware.hpp"
#include <mutex>
#include <memory>
#include <optional>
#include <vector>

namespace go2 {
constexpr std::size_t kObservationDim = 45;
using Action = Joints;
using Observation = std::array<float, kObservationDim>;
struct VelocityCommand { float vx{0}, vy{0}, yaw{0}; };

constexpr Joints kCrouchPose{0, 1.36F, -2.65F, 0, 1.36F, -2.65F,
                                0, 1.36F, -2.65F, 0, 1.36F, -2.65F};
constexpr Joints kStandKp{60, 80, 80, 60, 80, 80, 60, 80, 80, 60, 80, 80};
constexpr Joints kStandKd{5, 4, 4, 5, 4, 4, 5, 4, 4, 5, 4, 4};

enum class Mode { kPassive, kCrouch, kStand, kWalk, kRun, kTerrain,
    kWave, kRightFlip, kRollover, kForwardJump, kSit };
const char* mode_name(Mode);
Mode parse_mode(const std::string&);
bool mode_uses_policy(Mode);

enum class StatusLight { kGreen, kOrange, kRed };
struct StatusLightState {
    StatusLight color{StatusLight::kGreen};
    const char* fault{""};
};
StatusLightState status_light_state(Mode, const RobotState&, TimePoint now, bool input_alive,
    double imu_timeout = .30, double motor_timeout = .30);

class PolicyAdapter;
struct ControllerConfig {
    std::shared_ptr<const PolicyAdapter> policy_adapter;
    std::vector<Joints> sit_reference;
    double motor_soft_timeout{.22}, motor_hard_timeout{.30};
    // New-bigcore/new-smallcore reply loss is recoverable in the motor worker.
    // Keep the bounded freshness grace for all four legs; the field names are
    // retained for compatibility with existing read-only ROS parameters.
    bool rr_uart6_degraded{false};
    double rr_motor_soft_timeout{.22}, rr_motor_hard_timeout{.30};
    double imu_soft_timeout{.22}, imu_hard_timeout{.30}, command_timeout{.30};
    bool walking_tilt_protection{false};
    double stand_duration{1}, crouch_pose_duration{1}, crouch_duration{1};
    // Shared policy-side slew applies to web, terminal, SBUS, navigation and AI.
    // Lower-speed moving modes use a short ramp; fast running keeps its envelope.
    double move_linear_acceleration{4.0}, move_yaw_acceleration{8.0};
    double move_linear_deceleration{6.0}, move_yaw_deceleration{10.0};
    double run_linear_acceleration{1.5}, run_yaw_acceleration{2.0};
    double run_linear_deceleration{2.5}, run_yaw_deceleration{3.0};
    int maximum_temperature{77};
    float maximum_position{4}, maximum_velocity{50};
    Joints crouch_pose{-0.093852362F, 1.224573409F, -2.755849055F,
                    0.065076575F, 1.244820799F, -2.773433157F,
                    -0.329157191F, 1.299143482F, -2.835165058F,
                    0.331001033F, 1.297058558F, -2.815711514F};
};

struct ModeResult { bool accepted; std::string message; };
struct PolicyRequest {
    Mode mode;
    std::vector<float> observation;
    RobotState state;
    std::uint64_t policy_frame;
    std::uint64_t generation;
    TimePoint sampled_at;
};
struct ControllerStatus {
    Mode mode;
    bool protective_crouch;
    std::string reason;
    std::size_t rollover_step{0};
    bool rollover_step_running{false};
    std::size_t rollover_segments{0};
};

// ROS-free control state machine. Inference runs outside this mutex, so it
// cannot block the 1 ms safety loop. A generation token rejects obsolete work.
class Controller final {
    public:
    explicit Controller(ControllerConfig config);
    void set_velocity(VelocityCommand, TimePoint now);
    ModeResult set_mode(Mode, const RobotState&, TimePoint now, bool finish_motion = false);
    ModeResult step_rollover(const RobotState&, TimePoint now);
    Commands tick(const RobotState&, TimePoint now);
    std::optional<PolicyRequest> prepare_policy(const RobotState&, TimePoint now);
    void complete_policy(const PolicyRequest&, const Action&, TimePoint now);
    void fail_policy(const PolicyRequest&, const std::string& reason);
    ControllerStatus status() const;
    private:
    enum class Health { kFresh, kStale, kTooHot, kPositionOutOfRange, kUnsafe };
    Health motor_health(const RobotState&, TimePoint, std::string&,
                        bool enforce_position_limits = true) const;
    bool imu_fresh(const ImuSample&, TimePoint) const;
    void passive(const std::string&);
    void crouch(const RobotState&, TimePoint, std::string);
    void enter(Mode, const RobotState&, TimePoint);
    VelocityCommand ramp_velocity(VelocityCommand target, TimePoint now);
    void invalidate_policy();
    void reset_skill_progress();
    void finish_skill(const RobotState&, TimePoint);
    Commands remember(Commands);
    // Called only while a public entry point holds mutex_. An optional command
    // ends the tick immediately (brake/hold/transition); nullopt continues it.
    std::optional<Commands> check_motor_state(const RobotState&, TimePoint);
    std::optional<Commands> advance_stand_transition(const RobotState&, TimePoint);
    std::optional<Commands> check_policy_health(const RobotState&, TimePoint);
    void check_walking_tilt(const RobotState&, TimePoint);
    void advance_recovery(const RobotState&, TimePoint);
    Commands mode_commands(const RobotState&, TimePoint);

    // Policy preparation keeps mode transitions before observation assembly.
    VelocityCommand policy_command(TimePoint);
    bool policy_phase_ready(const RobotState&, TimePoint);
    void begin_flip_recovery(const RobotState&, TimePoint);
    PolicyRequest prepare_mode_observation(const RobotState&, VelocityCommand, TimePoint);

    // Core mode, command and watchdog state.
    ControllerConfig config_;
    mutable std::mutex mutex_;
    Mode mode_{Mode::kPassive};
    std::uint64_t generation_{0};
    VelocityCommand velocity_{};
    VelocityCommand ramped_velocity_{};
    TimePoint velocity_at_{}, transition_at_{}, policy_at_{};
    TimePoint ramped_velocity_at_{};
    std::optional<TimePoint> imu_stale_since_;
    std::optional<TimePoint> exit_gait_until_;
    bool terrain_to_walk_pending_{false};
    bool flip_walk_pending_{false};
    Joints initial_{}, target_{};
    std::size_t target_variant_ = 0;
    Action previous_{};
    Action previous_previous_{};
    Commands last_{};
    bool last_valid_{false}, target_valid_{false}, protective_{false};
    bool passive_crouch_{false};
    std::uint64_t protective_crouch_id_{0};
    bool passive_after_exit_{false};
    bool imu_latched_{false}, stale_hold_{false}, reseed_{false};
    bool pose_a_recovery_{false};
    std::optional<Mode> pending_mode_;
    // Finite motions and recovery stages.
    std::size_t rollover_step_{0};
    bool rollover_step_running_{false};
    TimePoint rollover_step_at_{};
    std::uint64_t policy_frame_{0};
    std::optional<float> sit_exit_start_;
    std::optional<TimePoint> sit_exit_stable_since_;
    unsigned wave_tick_{0U};
    bool wave_raised_{true};
    // Mode-specific observation histories and entry/exit gates.
    std::array<float, 225> cts_history_{};
    bool cts_history_initialized_{false};
    std::string inference_fault_, reason_;
};
} // namespace go2
#endif
