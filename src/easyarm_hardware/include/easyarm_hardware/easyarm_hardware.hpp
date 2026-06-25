#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "easyarm_interfaces/srv/get_debug_logger_status.hpp"
#include "easyarm_interfaces/srv/list_debug_logs.hpp"
#include "easyarm_interfaces/srv/set_debug_logger.hpp"
#include "easyarm_hardware/debug_logger.hpp"
#include "easyarm_dynamics/robot_model.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "robstride_can/robstride_can_driver.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"

namespace easyarm_hardware
{

using robstride_can::MotorType;
using robstride_can::RobstrideCanDriver;
using robstride_can::RunMode;

enum class MotorControlMode
{
  MotionControl,
  PositionCsp
};

enum class ControlMode
{
  Idle = 0,
  Position = 1
};

enum class FullCommandSource
{
  Hardware,
  Controller
};

struct JointConfig
{
  std::string name;
  uint8_t motor_id{0};
  MotorType motor_type{MotorType::RS00};
  double position_offset{0.0};
  double direction{1.0};
  double lower_limit{-6.28};
  double upper_limit{6.28};
  double kp{0.0};
  double kd{0.0};
};

struct DebugLoggerStatus
{
  bool active{false};
  std::string path;
  uint64_t written_count{0};
  uint64_t dropped_count{0};
};

class EasyArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EasyArmHardware)

  EasyArmHardware();
  ~EasyArmHardware() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool parse_joint_config(const hardware_interface::HardwareInfo & info);
  bool has_interface(const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name) const;
  MotorControlMode parse_control_mode(const std::string & value) const;
  const char * control_mode_name(MotorControlMode mode) const;
  ControlMode parse_hardware_control_mode(const std::string & value) const;
  bool try_parse_hardware_control_mode(const std::string & value, ControlMode & mode) const;
  const char * hardware_control_mode_name(ControlMode mode) const;
  MotorType parse_motor_type(const std::string & value) const;
  uint8_t parse_u8_parameter(const std::string & value, uint8_t default_value) const;
  double parse_double_parameter(const std::string & value, double default_value) const;
  bool parse_bool_parameter(const std::string & value, bool default_value) const;
  bool wait_for_robot_description(std::string & robot_description, std::string & message) const;
  void start_control_mode_node();
  void stop_control_mode_node();
  rcl_interfaces::msg::SetParametersResult on_control_mode_parameters(
    const std::vector<rclcpp::Parameter> & parameters);
  bool request_control_mode(ControlMode mode, std::string & message);
  void handle_set_debug_logger(
    const std::shared_ptr<easyarm_interfaces::srv::SetDebugLogger::Request> request,
    std::shared_ptr<easyarm_interfaces::srv::SetDebugLogger::Response> response);
  void handle_get_debug_logger_status(
    const std::shared_ptr<easyarm_interfaces::srv::GetDebugLoggerStatus::Request> request,
    std::shared_ptr<easyarm_interfaces::srv::GetDebugLoggerStatus::Response> response);
  void handle_list_debug_logs(
    const std::shared_ptr<easyarm_interfaces::srv::ListDebugLogs::Request> request,
    std::shared_ptr<easyarm_interfaces::srv::ListDebugLogs::Response> response);
  void apply_requested_control_mode();
  void apply_requested_debug_logger();
  bool apply_debug_logger_enabled(bool enabled, std::string & message);
  DebugLoggerStatus debug_logger_status() const;
  void fill_debug_logger_status(
    easyarm_interfaces::srv::SetDebugLogger::Response & response,
    bool success,
    const std::string & message) const;
  void fill_debug_logger_status(
    easyarm_interfaces::srv::GetDebugLoggerStatus::Response & response,
    bool success,
    const std::string & message) const;
  void reset_command_filters_to_current_state();
  bool switch_motor_mode(MotorControlMode mode);
  void sync_states_to_commands();
  void send_damping_before_disable();
  double clamp_joint_position(size_t joint_index, double position) const;
  void start_debug_logger();
  void stop_debug_logger();
  HardwareDebugSample make_debug_sample(const rclcpp::Duration & period);
  void fill_debug_joint_command(
    HardwareDebugSample & sample,
    size_t joint_index,
    double motor_position,
    double motor_velocity,
    double motor_torque,
    double kp,
    double kd,
    bool send_ok) const;
  void push_debug_sample(
    HardwareDebugSample & sample,
    std::chrono::steady_clock::time_point write_start,
    bool include_send_counts);

  rclcpp::Logger logger_{rclcpp::get_logger("easyarm_hardware")};

  std::unique_ptr<RobstrideCanDriver> can_driver_;
  std::unique_ptr<easyarm_dynamics::RobotModel> robot_model_;
  std::string can_interface_{"can0"};
  uint8_t host_can_id_{0xFD};
  std::string urdf_path_;

  std::vector<JointConfig> joint_configs_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_temperatures_;

  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_kps_;
  std::vector<double> hw_commands_kds_;
  std::vector<double> hw_commands_efforts_;

  double position_kp_{100.0};
  double position_kd_{4.0};
  double velocity_limit_{30.0};
  double smoothing_alpha_{0.8};
  double max_velocity_{2.0};
  double max_acceleration_{8.0};
  double control_period_{0.005};
  bool enable_gravity_compensation_{false};
  double gravity_compensation_scale_{1.0};
  double idle_kd_{4.0};
  double control_torque_limit_scale_{0.5};
  bool use_mock_hardware_{false};
  DebugLoggerConfig debug_logger_config_;
  DebugLogger debug_logger_;
  uint64_t debug_sequence_{0};
  mutable std::mutex debug_state_mutex_;
  std::mutex debug_request_mutex_;
  std::condition_variable debug_request_cv_;
  bool debug_request_pending_{false};
  bool debug_request_enabled_{false};
  bool debug_request_success_{true};
  std::string debug_request_message_{"OK"};
  uint64_t debug_request_generation_{0};
  uint64_t debug_applied_generation_{0};
  MotorControlMode desired_motor_mode_{MotorControlMode::MotionControl};
  MotorControlMode active_motor_mode_{MotorControlMode::MotionControl};
  ControlMode control_mode_{ControlMode::Position};
  std::atomic<int> requested_control_mode_{static_cast<int>(ControlMode::Position)};
  FullCommandSource full_command_source_{FullCommandSource::Hardware};

  rclcpp::Node::SharedPtr control_mode_node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr control_mode_param_callback_;
  rclcpp::Service<easyarm_interfaces::srv::SetDebugLogger>::SharedPtr set_debug_logger_service_;
  rclcpp::Service<easyarm_interfaces::srv::GetDebugLoggerStatus>::SharedPtr get_debug_logger_status_service_;
  rclcpp::Service<easyarm_interfaces::srv::ListDebugLogs>::SharedPtr list_debug_logs_service_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> control_mode_executor_;
  std::thread control_mode_executor_thread_;

  std::vector<double> smoothed_positions_;
  std::vector<double> smoothed_velocities_;
  std::vector<double> smoothed_accelerations_;
  std::vector<double> last_cmd_positions_;
  std::vector<double> filtered_cmd_velocities_;
  std::vector<double> velocity_ff_stage2_;
  std::vector<std::array<double, 4>> vel_ma_buffer_;
  std::vector<int> vel_ma_idx_;
  std::vector<int> velocity_settle_counter_;

  Eigen::VectorXd gravity_positions_;
  Eigen::VectorXd gravity_torques_;
};

}  // namespace easyarm_hardware
