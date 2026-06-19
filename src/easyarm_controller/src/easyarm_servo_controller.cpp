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
    auto_declare<double>("command_timeout_sec", command_timeout_sec_);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
EasyArmServoController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    interfaceNames(hardware_interface::HW_IF_POSITION)};
}

controller_interface::InterfaceConfiguration
EasyArmServoController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    interfaceNames(hardware_interface::HW_IF_POSITION)};
}

controller_interface::CallbackReturn EasyArmServoController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  command_timeout_sec_ = get_node()->get_parameter("command_timeout_sec").as_double();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must not be empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (command_timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'command_timeout_sec' must be positive");
    return controller_interface::CallbackReturn::ERROR;
  }

  hold_positions_.assign(joint_names_.size(), 0.0);
  last_velocities_.assign(joint_names_.size(), 0.0);
  last_accelerations_.assign(joint_names_.size(), 0.0);
  command_buffer_.initRT(CommandData{});

  array_command_subscriber_ = get_node()->create_subscription<ArrayCommandMsg>(
    "~/position_commands",
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
    "Configured EasyArmServoController with %zu joints, timeout %.3fs, topics '~/position_commands' and '~/joint_trajectory'",
    joint_names_.size(),
    command_timeout_sec_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EasyArmServoController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces, got %zu",
      joint_names_.size(),
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (state_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces, got %zu",
      joint_names_.size(),
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!readHoldPositionFromState()) {
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
    hold_positions_ = command->positions;
    has_last_velocities_ = command->has_velocities;
    has_last_accelerations_ = command->has_accelerations;
    if (has_last_velocities_) {
      last_velocities_ = command->velocities;
    }
    if (has_last_accelerations_) {
      last_accelerations_ = command->accelerations;
    }
  }

  writeHoldCommand();
  return controller_interface::return_type::OK;
}

std::vector<std::string> EasyArmServoController::interfaceNames(
  const std::string & interface_name) const
{
  std::vector<std::string> names;
  names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    names.push_back(joint_name + "/" + interface_name);
  }
  return names;
}

bool EasyArmServoController::readHoldPositionFromState()
{
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double position = state_interfaces_[i].get_value();
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
  const auto count = std::min(command_interfaces_.size(), hold_positions_.size());
  for (size_t i = 0; i < count; ++i) {
    command_interfaces_[i].set_value(hold_positions_[i]);
  }
}

}  // namespace easyarm_controller

PLUGINLIB_EXPORT_CLASS(
  easyarm_controller::EasyArmServoController,
  controller_interface::ControllerInterface)
