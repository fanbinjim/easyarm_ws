#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "robstride_can/robstride_can_driver.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

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
  MotorType parse_motor_type(const std::string & value) const;
  uint8_t parse_u8_parameter(const std::string & value, uint8_t default_value) const;
  double parse_double_parameter(const std::string & value, double default_value) const;
  bool switch_motor_mode(MotorControlMode mode);
  void sync_states_to_commands();
  void send_damping_before_disable();
  double clamp_joint_position(size_t joint_index, double position) const;

  rclcpp::Logger logger_{rclcpp::get_logger("easyarm_hardware")};

  std::unique_ptr<RobstrideCanDriver> can_driver_;
  std::string can_interface_{"can0"};
  uint8_t host_can_id_{0xFD};

  std::vector<JointConfig> joint_configs_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_temperatures_;

  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;

  double position_kp_{100.0};
  double position_kd_{4.0};
  double velocity_limit_{10.0};
  double smoothing_alpha_{0.8};
  double max_velocity_{2.0};
  double max_acceleration_{8.0};
  double control_period_{0.005};
  bool use_mock_hardware_{false};
  MotorControlMode desired_motor_mode_{MotorControlMode::MotionControl};
  MotorControlMode active_motor_mode_{MotorControlMode::MotionControl};

  std::vector<double> smoothed_positions_;
  std::vector<double> smoothed_velocities_;
  std::vector<double> smoothed_accelerations_;
  std::vector<double> last_cmd_positions_;
  std::vector<double> filtered_cmd_velocities_;
  std::vector<double> velocity_ff_stage2_;
  std::vector<std::array<double, 4>> vel_ma_buffer_;
  std::vector<int> vel_ma_idx_;
  std::vector<int> velocity_settle_counter_;
};

}  // namespace easyarm_hardware
