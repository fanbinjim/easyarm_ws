#include "easyarm_controller/easyarm_freedrive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iterator>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace easyarm_controller
{

controller_interface::CallbackReturn EasyArmFreedriveController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>(
      "command_interfaces",
      jointMotionControlInterfaceVector());
    auto_declare<std::vector<std::string>>(
      "state_interfaces",
      std::vector<std::string>{hardware_interface::HW_IF_POSITION});
    auto_declare<bool>("enable_gravity_compensation", enable_gravity_compensation_);
    auto_declare<double>("gravity_compensation_scale", gravity_compensation_scale_);
    auto_declare<double>("kd", kd_);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration EasyArmFreedriveController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    interfaceNames(command_interface_names_)};
}

controller_interface::InterfaceConfiguration EasyArmFreedriveController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    interfaceNames(state_interface_names_)};
}

controller_interface::CallbackReturn EasyArmFreedriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  command_interface_names_ = get_node()->get_parameter("command_interfaces").as_string_array();
  state_interface_names_ = get_node()->get_parameter("state_interfaces").as_string_array();
  enable_gravity_compensation_ = get_node()->get_parameter("enable_gravity_compensation").as_bool();
  gravity_compensation_scale_ = get_node()->get_parameter("gravity_compensation_scale").as_double();
  kd_ = get_node()->get_parameter("kd").as_double();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must not be empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!configureInterfaces()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (gravity_compensation_scale_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'gravity_compensation_scale' must be non-negative");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (kd_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'kd' must be non-negative");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!dynamics_provider_.configure(enable_gravity_compensation_, gravity_compensation_scale_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure dynamics provider");
    return controller_interface::CallbackReturn::ERROR;
  }

  commands_.assign(joint_names_.size(), JointMotionControlCommand{});
  for (auto & command : commands_) {
    command.velocity = 0.0;
    command.kp = 0.0;
    command.kd = kd_;
    command.effort = 0.0;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured EasyArmFreedriveController with %zu joints, kd=%.3f, gravity_scale=%.3f",
    joint_names_.size(),
    kd_,
    gravity_compensation_scale_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EasyArmFreedriveController::on_activate(
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
  if (!dynamics_provider_.initialize(joint_names_, get_node()->get_logger())) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!updateFreedriveCommandFromState()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  writeCommand();
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Activated EasyArmFreedriveController; keep hardware mode POSITION for this prototype");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EasyArmFreedriveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivated EasyArmFreedriveController");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type EasyArmFreedriveController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!updateFreedriveCommandFromState()) {
    return controller_interface::return_type::ERROR;
  }

  writeCommand();
  return controller_interface::return_type::OK;
}

std::vector<std::string> EasyArmFreedriveController::interfaceNames(
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

bool EasyArmFreedriveController::configureInterfaces()
{
  if (command_interface_names_ != jointMotionControlInterfaceVector()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "EasyArmFreedriveController requires fixed command interface order: position, velocity, kp, kd, effort");
    return false;
  }
  if (state_interface_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'state_interfaces' must not be empty");
    return false;
  }
  if (!std::any_of(
      state_interface_names_.begin(),
      state_interface_names_.end(),
      [](const auto & interface_name) { return interface_name == hardware_interface::HW_IF_POSITION; }))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "EasyArmFreedriveController requires position state interface");
    return false;
  }

  return true;
}

bool EasyArmFreedriveController::readCurrentPositions(std::vector<double> & positions) const
{
  positions.resize(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double position = state_interfaces_[stateIndex(i, hardware_interface::HW_IF_POSITION)].get_value();
    if (!std::isfinite(position)) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "State position for joint '%s' is not finite",
        joint_names_[i].c_str());
      return false;
    }
    positions[i] = position;
  }
  return true;
}

bool EasyArmFreedriveController::updateFreedriveCommandFromState()
{
  std::vector<double> positions;
  if (!readCurrentPositions(positions)) {
    return false;
  }

  std::vector<double> efforts;
  if (!dynamics_provider_.computeFeedforwardEffort(positions, efforts, get_node()->get_logger())) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Failed to compute freedrive feedforward effort; holding previous effort command");
    efforts.clear();
  }

  for (size_t i = 0; i < commands_.size(); ++i) {
    commands_[i].position = positions[i];
    commands_[i].velocity = 0.0;
    commands_[i].kp = 0.0;
    commands_[i].kd = kd_;
    if (efforts.size() == commands_.size()) {
      commands_[i].effort = efforts[i];
    }
  }
  return true;
}

void EasyArmFreedriveController::writeCommand()
{
  const auto count = std::min(joint_names_.size(), commands_.size());
  for (size_t i = 0; i < count; ++i) {
    command_interfaces_[commandIndex(i, hardware_interface::HW_IF_POSITION)].set_value(commands_[i].position);
    command_interfaces_[commandIndex(i, hardware_interface::HW_IF_VELOCITY)].set_value(commands_[i].velocity);
    command_interfaces_[commandIndex(i, kCommandInterfaceKp)].set_value(commands_[i].kp);
    command_interfaces_[commandIndex(i, kCommandInterfaceKd)].set_value(commands_[i].kd);
    command_interfaces_[commandIndex(i, hardware_interface::HW_IF_EFFORT)].set_value(commands_[i].effort);
  }
}

size_t EasyArmFreedriveController::commandIndex(
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

size_t EasyArmFreedriveController::stateIndex(
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
  easyarm_controller::EasyArmFreedriveController,
  controller_interface::ControllerInterface)
