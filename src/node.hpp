#ifndef NODE_HPP
#define NODE_HPP
#include "controller.hpp"
#include "hardware.hpp"
#include "policy.hpp"
#include <atomic>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <led.h>
#include "unitree_go2/srv/set_mode.hpp"
#include "unitree_go2/srv/calibration.hpp"

namespace go2 {
class Node final : public rclcpp::Node {
    public:
    Node();
    ~Node() override;
    private:
    void control_tick();
    void policy_tick();
    void publish_imu();
    void publish_joints();
    void publish_status();
    void update_lights();
    void update_input_control(TimePoint now);
    bool apply_lights();
    std::unique_ptr<led_dev, decltype(&led_free)> leds_{nullptr, led_free};
    std::string input_source_, light_fault_;
    StatusLight status_light_{StatusLight::kGreen};
    bool white_light_enabled_{false}, sbus_alive_{false}, web_alive_{false};
    TimePoint sbus_report_at_{}, web_report_at_{};
    double light_imu_timeout_{.30}, light_motor_timeout_{.30};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sbus_alive_sub_, web_alive_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sbus_slot_sub_;
    std::atomic<int> sbus_slot_{0};
    std::atomic<bool> rc_control_{false};
    TimePoint sbus_slot_at_{};
    double sbus_connection_timeout_{1.0};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr white_light_service_;
    rclcpp::TimerBase::SharedPtr light_timer_;
    std::unique_ptr<Hardware> hardware_;
    std::unique_ptr<Controller> controller_;
    std::unique_ptr<Policy> policy_;
    rclcpp::CallbackGroup::SharedPtr control_group_, policy_group_;
    rclcpp::TimerBase::SharedPtr control_timer_, policy_timer_, imu_timer_, joints_timer_, status_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rc_velocity_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr web_velocity_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_velocity_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ai_agent_velocity_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr direct_velocity_pub_;
    rclcpp::Service<unitree_go2::srv::SetMode>::SharedPtr mode_service_;
    rclcpp::Service<unitree_go2::srv::SetMode>::SharedPtr rc_mode_service_, web_mode_service_;
    rclcpp::Service<unitree_go2::srv::Calibration>::SharedPtr calibration_service_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    std::string robot_, board_profile_, backend_;
    std::string imu_frame_;
    double orientation_covariance_{.05}, angular_covariance_{.01}, acceleration_covariance_{.1};
    std::uint64_t imu_sequence_{0}, period_count_{0};
    std::uint64_t debug_control_ticks_{0}, debug_policy_requests_{0};
    std::uint64_t debug_policy_completed_{0}, debug_policy_failed_{0};
    TimePoint last_control_{}, last_report_{};
    TimePoint last_status_report_{};
    bool native_uart_debug_{false}, last_native_protective_{false};
    TimePoint last_native_uart_report_{};
    std::array<std::string, kNumGo2Legs> native_uart_devices_{};
    double period_sum_ms_{0}, period_max_ms_{0};
    std::atomic<double> inference_ms_{0};
    std::atomic<bool> navigation_mode_{false};
    std::atomic<bool> ai_agent_mode_{false};
    std::atomic<bool> calibration_active_{false};
    std::string last_status_;
    std::array<std::string, kNumGo2Legs> last_leg_errors_{};
    std::string last_debug_policy_mode_;
    Mode last_control_mode_{Mode::kPassive};
    // Optional CSV recorder (motion_log_path).  Writes target vs measured joint
    // motion plus the gyro and the attitude the free-locomotion family consumes,
    // so an oscillation can be analysed offline without extra tooling on the
    // robot.  Decimated to 200 Hz.
    std::ofstream motion_log_;
    std::uint64_t motion_log_tick_{0};
    TimePoint motion_log_start_{};
};
} // namespace go2

#endif  // NODE_HPP
