#include "easyarm_motion_server/moveit_servo_executor.hpp"

#include <chrono>
#include <future>
#include <vector>

namespace easyarm_motion_server
{
namespace
{
constexpr auto kServiceWaitTimeout = std::chrono::seconds(3);
constexpr auto kServiceCallTimeout = std::chrono::seconds(3);

builtin_interfaces::msg::Duration secondsToDuration(const double seconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(seconds);
  duration.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(duration.sec)) * 1e9);
  return duration;
}
}  // namespace

MoveItServoExecutor::MoveItServoExecutor(
  rclcpp::Node & node,
  const rclcpp::CallbackGroup::SharedPtr & callback_group)
: node_(node),
  callback_group_(callback_group),
  last_command_time_(node_.get_clock()->now())
{
  servo_command_timeout_sec_ =
    node_.declare_parameter<double>("servo_command_timeout_sec", servo_command_timeout_sec_);
  halt_message_count_ =
    node_.declare_parameter<int>("servo_halt_message_count", halt_message_count_);
  servo_controller_name_ =
    node_.declare_parameter<std::string>("servo_controller_name", "easyarm_servo_controller");
  trajectory_controller_name_ =
    node_.declare_parameter<std::string>("trajectory_controller_name", "arm_controller");

  switch_controller_client_ =
    node_.create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller",
      rmw_qos_profile_services_default,
      callback_group_);
  list_controllers_client_ =
    node_.create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers",
      rmw_qos_profile_services_default,
      callback_group_);
  start_servo_client_ =
    node_.create_client<std_srvs::srv::Trigger>(
      "/servo_node/start_servo",
      rmw_qos_profile_services_default,
      callback_group_);

  speedj_pub_ = node_.create_publisher<control_msgs::msg::JointJog>(
    "/servo_node/delta_joint_cmds",
    rclcpp::QoS(10));
  speedl_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(
    "/servo_node/delta_twist_cmds",
    rclcpp::QoS(10));
}

bool MoveItServoExecutor::isActive() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return servo_runtime_active_;
}

bool MoveItServoExecutor::enterServoRuntime(const std::string & task, std::string & message)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (servo_runtime_active_) {
      last_command_time_ = node_.get_clock()->now();
      return true;
    }
  }

  if (!start_servo_client_->wait_for_service(kServiceWaitTimeout)) {
    message = "/servo_node/start_servo service is not available";
    return false;
  }

  if (!switchControllers(servo_controller_name_, trajectory_controller_name_, message)) {
    return false;
  }

  if (!startMoveItServo(message)) {
    std::string restore_message;
    switchControllers(trajectory_controller_name_, servo_controller_name_, restore_message);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_task_ = task;
    servo_runtime_active_ = true;
    last_command_time_ = node_.get_clock()->now();
  }

  RCLCPP_INFO(node_.get_logger(), "Entered SERVO runtime with %s", task.c_str());
  return true;
}

bool MoveItServoExecutor::exitServoRuntime(std::string & message)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!servo_runtime_active_) {
      message = "SERVO runtime is not active";
      return true;
    }
    if (servo_runtime_exiting_) {
      message = "SERVO runtime is already exiting";
      return true;
    }
    servo_runtime_exiting_ = true;
  }

  publishZeroCommands();

  if (!switchControllers(trajectory_controller_name_, servo_controller_name_, message)) {
    std::lock_guard<std::mutex> lock(mutex_);
    servo_runtime_exiting_ = false;
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    servo_runtime_active_ = false;
    servo_runtime_exiting_ = false;
    active_task_.clear();
  }

  RCLCPP_INFO(node_.get_logger(), "Exited SERVO runtime");
  message = "SERVO runtime stopped";
  return true;
}

void MoveItServoExecutor::stop()
{
  std::string message;
  if (!exitServoRuntime(message)) {
    RCLCPP_ERROR(node_.get_logger(), "Failed to stop SERVO runtime: %s", message.c_str());
  }
}

void MoveItServoExecutor::forwardSpeedJ(const control_msgs::msg::JointJog & command)
{
  speedj_pub_->publish(command);
  std::lock_guard<std::mutex> lock(mutex_);
  last_command_time_ = node_.get_clock()->now();
}

void MoveItServoExecutor::forwardSpeedL(const geometry_msgs::msg::TwistStamped & command)
{
  speedl_pub_->publish(command);
  std::lock_guard<std::mutex> lock(mutex_);
  last_command_time_ = node_.get_clock()->now();
}

void MoveItServoExecutor::update()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!servo_runtime_active_ || servo_runtime_exiting_ || !hasTimedOut(node_.get_clock()->now())) {
      return;
    }
  }

  std::string message;
  RCLCPP_INFO(
    node_.get_logger(),
    "SERVO command timeout, returning to %s",
    trajectory_controller_name_.c_str());
  if (!exitServoRuntime(message)) {
    RCLCPP_ERROR(node_.get_logger(), "Failed to exit SERVO runtime after timeout: %s", message.c_str());
  }
}

bool MoveItServoExecutor::switchControllers(
  const std::string & activate,
  const std::string & deactivate,
  std::string & message)
{
  const auto activate_state = controllerState(activate, message);
  if (!activate_state.has_value()) {
    return false;
  }
  const auto deactivate_state = controllerState(deactivate, message);
  if (!deactivate_state.has_value()) {
    return false;
  }
  std::vector<std::string> activate_controllers;
  std::vector<std::string> deactivate_controllers;

  if (*activate_state != "active") {
    activate_controllers.push_back(activate);
  }
  if (*deactivate_state == "active") {
    deactivate_controllers.push_back(deactivate);
  }

  if (activate_controllers.empty() && deactivate_controllers.empty()) {
    return true;
  }

  if (!switch_controller_client_->wait_for_service(kServiceWaitTimeout)) {
    message = "/controller_manager/switch_controller service is not available";
    return false;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = activate_controllers;
  request->deactivate_controllers = deactivate_controllers;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = secondsToDuration(3.0);

  auto future = switch_controller_client_->async_send_request(request);
  if (future.wait_for(kServiceCallTimeout) != std::future_status::ready) {
    message = "Timeout switching controllers: activate " + activate + ", deactivate " + deactivate;
    return false;
  }

  const auto response = future.get();
  if (!response->ok) {
    message = "Failed to switch controllers: activate " + activate + ", deactivate " + deactivate;
    return false;
  }

  return true;
}

std::optional<std::string> MoveItServoExecutor::controllerState(
  const std::string & controller_name,
  std::string & message)
{
  if (!list_controllers_client_->wait_for_service(kServiceWaitTimeout)) {
    message = "/controller_manager/list_controllers service is not available";
    return std::nullopt;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = list_controllers_client_->async_send_request(request);
  if (future.wait_for(kServiceCallTimeout) != std::future_status::ready) {
    message = "Timeout listing controllers";
    return std::nullopt;
  }

  const auto response = future.get();
  for (const auto & controller : response->controller) {
    if (controller.name == controller_name) {
      return controller.state;
    }
  }

  message = "Controller '" + controller_name + "' is not loaded";
  return std::nullopt;
}

bool MoveItServoExecutor::startMoveItServo(std::string & message)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = start_servo_client_->async_send_request(request);
  if (future.wait_for(kServiceCallTimeout) != std::future_status::ready) {
    message = "Timeout calling /servo_node/start_servo";
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    message = response->message.empty() ? "/servo_node/start_servo failed" : response->message;
    return false;
  }

  return true;
}

void MoveItServoExecutor::publishZeroCommands()
{
  for (int index = 0; index < halt_message_count_; ++index) {
    publishZeroJointJog();
    publishZeroTwist();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
}

void MoveItServoExecutor::publishZeroJointJog()
{
  control_msgs::msg::JointJog message;
  message.header.stamp = node_.get_clock()->now();
  message.header.frame_id = "base_link";
  message.joint_names = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
  message.velocities.assign(6, 0.0);
  speedj_pub_->publish(message);
}

void MoveItServoExecutor::publishZeroTwist()
{
  geometry_msgs::msg::TwistStamped message;
  message.header.stamp = node_.get_clock()->now();
  message.header.frame_id = "base_link";
  speedl_pub_->publish(message);
}

bool MoveItServoExecutor::hasTimedOut(const rclcpp::Time & now) const
{
  return (now - last_command_time_).seconds() > servo_command_timeout_sec_;
}

}  // namespace easyarm_motion_server
