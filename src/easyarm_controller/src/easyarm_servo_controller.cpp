#include "easyarm_controller/easyarm_servo_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace easyarm_controller
{
controller_interface::CallbackReturn EasyArmServoController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>(
      "command_interfaces",
      std::vector<std::string>{hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_EFFORT});
    auto_declare<std::vector<std::string>>(
      "state_interfaces",
      std::vector<std::string>{hardware_interface::HW_IF_POSITION});
    auto_declare<double>("command_timeout_sec", command_timeout_sec_);
    auto_declare<bool>("enable_gravity_compensation", enable_gravity_compensation_);
    auto_declare<double>("gravity_compensation_scale", gravity_compensation_scale_);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration EasyArmServoController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    interfaceNames(command_interface_names_)};
}

controller_interface::InterfaceConfiguration EasyArmServoController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    interfaceNames(state_interface_names_)};
}

controller_interface::CallbackReturn EasyArmServoController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  command_interface_names_ = get_node()->get_parameter("command_interfaces").as_string_array();
  state_interface_names_ = get_node()->get_parameter("state_interfaces").as_string_array();
  command_timeout_sec_ = get_node()->get_parameter("command_timeout_sec").as_double();
  enable_gravity_compensation_ = get_node()->get_parameter("enable_gravity_compensation").as_bool();
  gravity_compensation_scale_ = get_node()->get_parameter("gravity_compensation_scale").as_double();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must not be empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!configureInterfaces()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (command_timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'command_timeout_sec' must be positive");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (gravity_compensation_scale_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'gravity_compensation_scale' must be non-negative");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!dynamics_provider_.configure(enable_gravity_compensation_, gravity_compensation_scale_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure dynamics provider");
    return controller_interface::CallbackReturn::ERROR;
  }

  hold_positions_.assign(joint_names_.size(), 0.0);
  hold_feedforward_efforts_.assign(joint_names_.size(), 0.0);
  last_velocities_.assign(joint_names_.size(), 0.0);
  last_accelerations_.assign(joint_names_.size(), 0.0);
  command_buffer_.initRT(CommandData{});

  array_command_subscriber_ = get_node()->create_subscription<ArrayCommandMsg>(
    "~/joint_positions",
    rclcpp::SystemDefaultsQoS(),
    [this](const ArrayCommandMsg::SharedPtr message) {
      arrayCommandCallback(message);
    });

  trajectory_command_subscriber_ = get_node()->create_subscription<TrajectoryCommandMsg>(
    "~/joint_trajectory",
    rclcpp::SystemDefaultsQoS(),
    [this](const TrajectoryCommandMsg::SharedPtr message) {
      trajectoryCommandCallback(message);
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured EasyArmServoController with %zu joints, timeout %.3fs, topics '~/joint_positions' and '~/joint_trajectory'",
    joint_names_.size(),
    command_timeout_sec_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EasyArmServoController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != joint_names_.size() * command_interface_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces, got %zu",
      joint_names_.size() * command_interface_names_.size(),
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (state_interfaces_.size() != joint_names_.size() * state_interface_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces, got %zu",
      joint_names_.size() * state_interface_names_.size(),
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!readHoldPositionFromState()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!dynamics_provider_.initialize(joint_names_, get_node()->get_logger())) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!dynamics_provider_.computeFeedforwardEffort(
      hold_positions_,
      hold_feedforward_efforts_,
      get_node()->get_logger()))
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  command_buffer_.writeFromNonRT(CommandData{});
  writeHoldCommand();

  RCLCPP_INFO(get_node()->get_logger(), "Activated EasyArmServoController");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EasyArmServoController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  command_buffer_.writeFromNonRT(CommandData{});
  RCLCPP_INFO(get_node()->get_logger(), "Deactivated EasyArmServoController");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type EasyArmServoController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  const auto command = command_buffer_.readFromRT();

  if (command && command->has_command && !commandTimedOut(time, *command)) {
    // 第一版将 MoveIt Servo 输出的关节 position 作为目标 setpoint 直通保存。
    hold_positions_ = command->positions;
    if (!dynamics_provider_.computeFeedforwardEffort(
        hold_positions_,
        hold_feedforward_efforts_,
        get_node()->get_logger()))
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Failed to compute feedforward effort; holding previous effort command");
    }
    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      500,
      "feedforward_effort Nm: [%.6f %.6f %.6f %.6f %.6f %.6f]",
      hold_feedforward_efforts_.size() > 0 ? hold_feedforward_efforts_[0] : 0.0,
      hold_feedforward_efforts_.size() > 1 ? hold_feedforward_efforts_[1] : 0.0,
      hold_feedforward_efforts_.size() > 2 ? hold_feedforward_efforts_[2] : 0.0,
      hold_feedforward_efforts_.size() > 3 ? hold_feedforward_efforts_[3] : 0.0,
      hold_feedforward_efforts_.size() > 4 ? hold_feedforward_efforts_[4] : 0.0,
      hold_feedforward_efforts_.size() > 5 ? hold_feedforward_efforts_[5] : 0.0);

    // velocity/acceleration 是上游轨迹输入；当前只缓存，后续用于计算完整动力学 effort。
    has_last_velocities_ = command->has_velocities;
    has_last_accelerations_ = command->has_accelerations;
    if (has_last_velocities_) {
      last_velocities_ = command->velocities;
    }
    if (has_last_accelerations_) {
      last_accelerations_ = command->accelerations;
    }
  }

  // 将最新目标写入 ros2_control command interfaces，由 hardware::write() 下发到电机。
  writeHoldCommand();
  return controller_interface::return_type::OK;
}

std::vector<std::string> EasyArmServoController::interfaceNames(
  const std::vector<std::string> & interface_names) const
{
  std::vector<std::string> names;
  names.reserve(joint_names_.size() * interface_names.size());
  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_name : interface_names) {
      names.push_back(joint_name + "/" + interface_name);
    }
  }
  return names;
}

bool EasyArmServoController::configureInterfaces()
{
  if (command_interface_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'command_interfaces' must not be empty");
    return false;
  }
  if (state_interface_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'state_interfaces' must not be empty");
    return false;
  }

  const auto is_supported_interface = [](const std::string & interface_name) {
    return interface_name == hardware_interface::HW_IF_POSITION ||
      interface_name == hardware_interface::HW_IF_VELOCITY ||
      interface_name == hardware_interface::HW_IF_EFFORT;
  };
  const auto has_duplicate = [](const std::vector<std::string> & interfaces) {
    for (size_t i = 0; i < interfaces.size(); ++i) {
      for (size_t j = i + 1; j < interfaces.size(); ++j) {
        if (interfaces[i] == interfaces[j]) {
          return true;
        }
      }
    }
    return false;
  };

  if (has_duplicate(command_interface_names_) || has_duplicate(state_interface_names_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Interface lists must not contain duplicates");
    return false;
  }
  for (const auto & interface_name : command_interface_names_) {
    if (!is_supported_interface(interface_name)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Unsupported command interface '%s'", interface_name.c_str());
      return false;
    }
  }
  for (const auto & interface_name : state_interface_names_) {
    if (!is_supported_interface(interface_name)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Unsupported state interface '%s'", interface_name.c_str());
      return false;
    }
  }

  if (!hasConfiguredCommandInterface(hardware_interface::HW_IF_POSITION)) {
    RCLCPP_ERROR(get_node()->get_logger(), "EasyArmServoController requires position command interface");
    return false;
  }
  if (hasConfiguredCommandInterface(hardware_interface::HW_IF_VELOCITY)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Velocity command interface is configurable but not implemented yet; remove 'velocity' for now");
    return false;
  }
  if (!std::any_of(
      state_interface_names_.begin(),
      state_interface_names_.end(),
      [](const auto & interface_name) { return interface_name == hardware_interface::HW_IF_POSITION; }))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "EasyArmServoController requires position state interface");
    return false;
  }

  return true;
}

bool EasyArmServoController::hasConfiguredCommandInterface(const std::string & interface_name) const
{
  return std::any_of(
    command_interface_names_.begin(),
    command_interface_names_.end(),
    [&interface_name](const auto & configured_interface) {
      return configured_interface == interface_name;
    });
}

bool EasyArmServoController::readHoldPositionFromState()
{
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double position = state_interfaces_[stateIndex(i, hardware_interface::HW_IF_POSITION)].get_value();
    if (!std::isfinite(position)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "State position for joint '%s' is not finite",
        joint_names_[i].c_str());
      return false;
    }
    hold_positions_[i] = position;
  }
  return true;
}

void EasyArmServoController::arrayCommandCallback(const ArrayCommandMsg::SharedPtr message)
{
  CommandData command;
  if (!parseArrayCommand(*message, command)) {
    return;
  }
  command.stamp = get_node()->get_clock()->now();
  command.source = "Float64MultiArray";
  command_buffer_.writeFromNonRT(command);
}

void EasyArmServoController::trajectoryCommandCallback(
  const TrajectoryCommandMsg::SharedPtr message)
{
  CommandData command;
  if (!parseTrajectoryCommand(*message, command)) {
    return;
  }
  command.stamp = get_node()->get_clock()->now();
  command.source = "JointTrajectory";
  command_buffer_.writeFromNonRT(command);
}

bool EasyArmServoController::parseArrayCommand(
  const ArrayCommandMsg & message,
  CommandData & command) const
{
  const auto joint_count = joint_names_.size();
  const auto command_size = message.data.size();
  if (command_size != joint_count) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Ignoring Float64MultiArray servo command with invalid size %zu; expected %zu position values",
      command_size,
      joint_count);
    return false;
  }

  command.positions.resize(joint_count);
  for (size_t i = 0; i < joint_count; ++i) {
    const double position = message.data[i];
    if (!std::isfinite(position)) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Ignoring servo command with non-finite position at index %zu",
        i);
      return false;
    }
    command.positions[i] = position;
  }

  command.has_command = true;
  command.has_velocities = false;
  command.has_accelerations = false;
  return true;
}

bool EasyArmServoController::parseTrajectoryCommand(
  const TrajectoryCommandMsg & message,
  CommandData & command) const
{
  const auto joint_count = joint_names_.size();
  if (message.points.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Ignoring JointTrajectory servo command with no points");
    return false;
  }

  std::vector<int> joint_index_map(joint_count, -1);
  for (size_t i = 0; i < joint_count; ++i) {
    const auto it = std::find(message.joint_names.begin(), message.joint_names.end(), joint_names_[i]);
    if (it == message.joint_names.end()) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Ignoring JointTrajectory servo command missing joint '%s'",
        joint_names_[i].c_str());
      return false;
    }
    joint_index_map[i] = static_cast<int>(std::distance(message.joint_names.begin(), it));
  }

  const auto & point = message.points.front();
  if (!copyTrajectoryField(point.positions, joint_index_map, "positions", command.positions)) {
    return false;
  }

  command.has_command = true;
  command.has_velocities = false;
  command.has_accelerations = false;

  if (!point.velocities.empty()) {
    if (!copyTrajectoryField(point.velocities, joint_index_map, "velocities", command.velocities)) {
      return false;
    }
    command.has_velocities = true;
  }

  if (!point.accelerations.empty()) {
    if (!copyTrajectoryField(point.accelerations, joint_index_map, "accelerations", command.accelerations)) {
      return false;
    }
    command.has_accelerations = true;
  }

  return true;
}

bool EasyArmServoController::copyTrajectoryField(
  const std::vector<double> & input,
  const std::vector<int> & joint_index_map,
  const char * field_name,
  std::vector<double> & output) const
{
  if (input.size() < joint_names_.size()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Ignoring JointTrajectory servo command with %zu %s values; expected at least %zu",
      input.size(),
      field_name,
      joint_names_.size());
    return false;
  }

  output.resize(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto index = joint_index_map[i];
    if (index < 0 || static_cast<size_t>(index) >= input.size()) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Ignoring JointTrajectory servo command with invalid %s index for joint '%s'",
        field_name,
        joint_names_[i].c_str());
      return false;
    }

    const double value = input[static_cast<size_t>(index)];
    if (!std::isfinite(value)) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Ignoring JointTrajectory servo command with non-finite %s for joint '%s'",
        field_name,
        joint_names_[i].c_str());
      return false;
    }
    output[i] = value;
  }

  return true;
}

bool EasyArmServoController::commandTimedOut(
  const rclcpp::Time & time,
  const CommandData & command) const
{
  return (time - command.stamp).seconds() > command_timeout_sec_;
}

void EasyArmServoController::writeHoldCommand()
{
  const auto count = std::min(joint_names_.size(), hold_positions_.size());
  for (size_t i = 0; i < count; ++i) {
    command_interfaces_[commandIndex(i, hardware_interface::HW_IF_POSITION)].set_value(hold_positions_[i]);
    if (hasConfiguredCommandInterface(hardware_interface::HW_IF_EFFORT)) {
      command_interfaces_[commandIndex(i, hardware_interface::HW_IF_EFFORT)].set_value(hold_feedforward_efforts_[i]);
    }
  }
}

size_t EasyArmServoController::commandIndex(
  const size_t joint_index,
  const std::string & interface_name) const
{
  const auto it = std::find(command_interface_names_.begin(), command_interface_names_.end(), interface_name);
  if (it == command_interface_names_.end()) {
    return command_interfaces_.size();
  }
  return joint_index * command_interface_names_.size() +
    static_cast<size_t>(std::distance(command_interface_names_.begin(), it));
}

size_t EasyArmServoController::stateIndex(
  const size_t joint_index,
  const std::string & interface_name) const
{
  const auto it = std::find(state_interface_names_.begin(), state_interface_names_.end(), interface_name);
  if (it == state_interface_names_.end()) {
    return state_interfaces_.size();
  }
  return joint_index * state_interface_names_.size() +
    static_cast<size_t>(std::distance(state_interface_names_.begin(), it));
}

}  // namespace easyarm_controller

PLUGINLIB_EXPORT_CLASS(
  easyarm_controller::EasyArmServoController,
  controller_interface::ControllerInterface)
