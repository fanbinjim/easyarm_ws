#include "easyarm_motion_server/motion_server_node.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <thread>
#include <vector>

namespace easyarm_motion_server
{
namespace
{
constexpr auto kControllerServiceWaitTimeout = std::chrono::seconds(3);
constexpr auto kControllerServiceCallTimeout = std::chrono::seconds(3);

builtin_interfaces::msg::Duration secondsToDuration(const double seconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(seconds);
  duration.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(duration.sec)) * 1e9);
  return duration;
}
}  // namespace

MotionServerNode::MotionServerNode(const rclcpp::NodeOptions & options)
: Node("easyarm_motion_server", options)
{
  context_.planning_group = declare_parameter<std::string>("planning_group", context_.planning_group);
  context_.ee_link = declare_parameter<std::string>("ee_link", context_.ee_link);
  context_.planning_frame = declare_parameter<std::string>("planning_frame", context_.planning_frame);
  context_.default_velocity_scale =
    declare_parameter<double>("default_velocity_scale", context_.default_velocity_scale);
  context_.default_acceleration_scale =
    declare_parameter<double>("default_acceleration_scale", context_.default_acceleration_scale);
  context_.movej_planner_id = declare_parameter<std::string>("movej_planner_id", context_.movej_planner_id);
  context_.movel_planner_id = declare_parameter<std::string>("movel_planner_id", context_.movel_planner_id);
  context_.planning_pipeline_id =
    declare_parameter<std::string>("planning_pipeline_id", context_.planning_pipeline_id);
  context_.joint_state_wait_timeout_sec =
    declare_parameter<double>("joint_state_wait_timeout", context_.joint_state_wait_timeout_sec);
  context_.max_joint_state_age_sec =
    declare_parameter<double>("max_joint_state_age", context_.max_joint_state_age_sec);
  freedrive_controller_name_ =
    declare_parameter<std::string>("freedrive_controller_name", freedrive_controller_name_);

  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  joint_state_cache_ = std::make_unique<JointStateCache>(*this, context_);
  hardware_mode_client_ = std::make_unique<HardwareModeClient>(*this, callback_group_);
  hold_trajectory_sender_ = std::make_unique<HoldTrajectorySender>(*this, context_, *joint_state_cache_, callback_group_);
  moveit_executor_ = std::make_unique<MoveItMotionExecutor>(*this, context_);
  moveit_servo_runtime_ = std::make_unique<MoveItServoRuntime>(*this, callback_group_);
  trajectory_controller_name_ = get_parameter("trajectory_controller_name").as_string();
  position_servo_executor_ =
    std::make_unique<PositionServoExecutor>(*this, context_, *joint_state_cache_, *moveit_servo_runtime_);
  switch_controller_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller",
    rmw_qos_profile_services_default,
    callback_group_);
  list_controllers_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers",
    rmw_qos_profile_services_default,
    callback_group_);

  movej_server_ = rclcpp_action::create_server<MoveJ>(
    this,
    "/easyarm/movej",
    std::bind(&MotionServerNode::handleMoveJGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MotionServerNode::handleMoveJCancel, this, std::placeholders::_1),
    std::bind(&MotionServerNode::handleMoveJAccepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    callback_group_);

  movel_server_ = rclcpp_action::create_server<MoveL>(
    this,
    "/easyarm/movel",
    std::bind(&MotionServerNode::handleMoveLGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MotionServerNode::handleMoveLCancel, this, std::placeholders::_1),
    std::bind(&MotionServerNode::handleMoveLAccepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    callback_group_);

  move_named_state_server_ = rclcpp_action::create_server<MoveNamedState>(
    this,
    "/easyarm/move_named_state",
    std::bind(&MotionServerNode::handleMoveNamedStateGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MotionServerNode::handleMoveNamedStateCancel, this, std::placeholders::_1),
    std::bind(&MotionServerNode::handleMoveNamedStateAccepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    callback_group_);

  set_mode_service_ = create_service<easyarm_interfaces::srv::SetMode>(
    "/easyarm/set_mode",
    std::bind(
      &MotionServerNode::handleSetMode,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group_);

  stop_service_ = create_service<easyarm_interfaces::srv::Stop>(
    "/easyarm/stop",
    std::bind(
      &MotionServerNode::handleStop,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group_);

  get_state_service_ = create_service<easyarm_interfaces::srv::GetState>(
    "/easyarm/get_state",
    std::bind(
      &MotionServerNode::handleGetState,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group_);

  get_joints_service_ = create_service<easyarm_interfaces::srv::GetJoints>(
    "/easyarm/get_joints",
    std::bind(
      &MotionServerNode::handleGetJoints,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group_);

  get_pose_service_ = create_service<easyarm_interfaces::srv::GetPose>(
    "/easyarm/get_pose",
    std::bind(
      &MotionServerNode::handleGetPose,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group_);

  list_named_state_service_ = create_service<easyarm_interfaces::srv::ListNamedState>(
    "/easyarm/list_named_state",
    std::bind(
      &MotionServerNode::handleListNamedState,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group_);

  speedj_sub_ = create_subscription<control_msgs::msg::JointJog>(
    "/easyarm/speedj_cmd",
    rclcpp::QoS(10),
    std::bind(&MotionServerNode::handleSpeedJCommand, this, std::placeholders::_1));

  speedl_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/easyarm/speedl_cmd",
    rclcpp::QoS(10),
    std::bind(&MotionServerNode::handleSpeedLCommand, this, std::placeholders::_1));

  servoj_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/easyarm/servoj_cmd",
    rclcpp::QoS(10),
    std::bind(&MotionServerNode::handleServoJCommand, this, std::placeholders::_1));

  servol_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/easyarm/servol_cmd",
    rclcpp::QoS(10),
    std::bind(&MotionServerNode::handleServoLCommand, this, std::placeholders::_1));

  const auto servo_rate_hz = std::max(1.0, get_parameter("position_servo_rate_hz").as_double());
  const auto servo_timer_period_ms = std::max(1, static_cast<int>(1000.0 / servo_rate_hz));
  servo_timer_ = create_wall_timer(
    std::chrono::milliseconds(servo_timer_period_ms),
    std::bind(&MotionServerNode::handleServoTimer, this),
    callback_group_);

  RCLCPP_INFO(
    get_logger(),
    "EasyArm motion server ready: group=%s, ee_link=%s, frame=%s, pipeline=%s",
    context_.planning_group.c_str(),
    context_.ee_link.c_str(),
    context_.planning_frame.c_str(),
    context_.planning_pipeline_id.c_str());
}

void MotionServerNode::initializeMoveGroup()
{
  moveit_executor_->initialize(shared_from_this());
  position_servo_executor_->initialize(shared_from_this());
}

rclcpp_action::GoalResponse MotionServerNode::handleMoveJGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MoveJ::Goal> goal)
{
  if (goal->joints.size() != context_.joint_names.size()) {
    RCLCPP_WARN(get_logger(), "Reject MoveJ: expected 6 joints, got %zu", goal->joints.size());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (busy_.load()) {
    const auto task = activeTaskSnapshot();
    RCLCPP_WARN(get_logger(), "Reject MoveJ: server is busy with %s", task.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionServerNode::handleMoveJCancel(const std::shared_ptr<GoalHandleMoveJ>)
{
  stop_requested_.store(true);
  moveit_executor_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionServerNode::handleMoveJAccepted(const std::shared_ptr<GoalHandleMoveJ> goal_handle)
{
  std::thread{std::bind(&MotionServerNode::executeMoveJ, this, goal_handle)}.detach();
}

rclcpp_action::GoalResponse MotionServerNode::handleMoveLGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MoveL::Goal>)
{
  if (busy_.load()) {
    const auto task = activeTaskSnapshot();
    RCLCPP_WARN(get_logger(), "Reject MoveL: server is busy with %s", task.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionServerNode::handleMoveLCancel(const std::shared_ptr<GoalHandleMoveL>)
{
  stop_requested_.store(true);
  moveit_executor_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionServerNode::handleMoveLAccepted(const std::shared_ptr<GoalHandleMoveL> goal_handle)
{
  std::thread{std::bind(&MotionServerNode::executeMoveL, this, goal_handle)}.detach();
}

rclcpp_action::GoalResponse MotionServerNode::handleMoveNamedStateGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MoveNamedState::Goal> goal)
{
  if (goal->name.empty()) {
    RCLCPP_WARN(get_logger(), "Reject MoveNamedState: empty named state");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (busy_.load()) {
    const auto task = activeTaskSnapshot();
    RCLCPP_WARN(get_logger(), "Reject MoveNamedState: server is busy with %s", task.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionServerNode::handleMoveNamedStateCancel(
  const std::shared_ptr<GoalHandleMoveNamedState>)
{
  stop_requested_.store(true);
  moveit_executor_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionServerNode::handleMoveNamedStateAccepted(
  const std::shared_ptr<GoalHandleMoveNamedState> goal_handle)
{
  std::thread{std::bind(&MotionServerNode::executeMoveNamedState, this, goal_handle)}.detach();
}

void MotionServerNode::executeMoveJ(const std::shared_ptr<GoalHandleMoveJ> goal_handle)
{
  auto result = std::make_shared<MoveJ::Result>();
  const auto goal = goal_handle->get_goal();

  if (!claimTask("MoveJ", result->message)) {
    result->success = false;
    goal_handle->abort(result);
    return;
  }

  publishMoveJFeedback(goal_handle, "preparing");

  bool success = false;
  std::string message;
  try {
    success = prepareMotion(message) &&
      moveit_executor_->runMoveJ(
        *goal,
        [goal_handle]() { return goal_handle->is_canceling(); },
        [this, goal_handle](const std::string & state) { publishMoveJFeedback(goal_handle, state); },
        message);
  } catch (const std::exception & exception) {
    message = exception.what();
    success = false;
  }

  if (goal_handle->is_canceling() || stop_requested_.load()) {
    result->success = false;
    result->message = "MoveJ canceled";
    goal_handle->canceled(result);
    releaseTask();
    return;
  }

  result->success = success;
  result->message = message;
  if (success) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
  releaseTask();
}

void MotionServerNode::executeMoveL(const std::shared_ptr<GoalHandleMoveL> goal_handle)
{
  auto result = std::make_shared<MoveL::Result>();
  const auto goal = goal_handle->get_goal();

  if (!claimTask("MoveL", result->message)) {
    result->success = false;
    goal_handle->abort(result);
    return;
  }

  publishMoveLFeedback(goal_handle, "preparing");

  bool success = false;
  std::string message;
  try {
    success = prepareMotion(message) &&
      moveit_executor_->runMoveL(
        *goal,
        [goal_handle]() { return goal_handle->is_canceling(); },
        [this, goal_handle](const std::string & state) { publishMoveLFeedback(goal_handle, state); },
        message);
  } catch (const std::exception & exception) {
    message = exception.what();
    success = false;
  }

  if (goal_handle->is_canceling() || stop_requested_.load()) {
    result->success = false;
    result->message = "MoveL canceled";
    goal_handle->canceled(result);
    releaseTask();
    return;
  }

  result->success = success;
  result->message = message;
  if (success) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
  releaseTask();
}

void MotionServerNode::executeMoveNamedState(const std::shared_ptr<GoalHandleMoveNamedState> goal_handle)
{
  auto result = std::make_shared<MoveNamedState::Result>();
  const auto goal = goal_handle->get_goal();

  if (!claimTask("MoveNamedState", result->message)) {
    result->success = false;
    goal_handle->abort(result);
    return;
  }

  publishMoveNamedStateFeedback(goal_handle, "preparing");

  bool success = false;
  std::string message;
  try {
    success = prepareMotion(message) &&
      moveit_executor_->runMoveNamedState(
        *goal,
        [goal_handle]() { return goal_handle->is_canceling(); },
        [this, goal_handle](const std::string & state) { publishMoveNamedStateFeedback(goal_handle, state); },
        message);
  } catch (const std::exception & exception) {
    message = exception.what();
    success = false;
  }

  if (goal_handle->is_canceling() || stop_requested_.load()) {
    result->success = false;
    result->message = "MoveNamedState canceled";
    goal_handle->canceled(result);
    releaseTask();
    return;
  }

  result->success = success;
  result->message = message;
  if (success) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
  releaseTask();
}

bool MotionServerNode::claimTask(const std::string & task, std::string & message)
{
  bool expected = false;
  if (!busy_.compare_exchange_strong(expected, true)) {
    message = "Motion server is busy with " + activeTaskSnapshot();
    return false;
  }
  stop_requested_.store(false);
  std::lock_guard<std::mutex> lock(state_mutex_);
  active_task_ = task;
  return true;
}

void MotionServerNode::releaseTask()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  active_task_.clear();
  stop_requested_.store(false);
  busy_.store(false);
}

bool MotionServerNode::prepareMotion(std::string & message)
{
  if (!moveit_executor_->isInitialized()) {
    message = "MoveGroupInterface is not initialized";
    return false;
  }
  if (!exitFreeDriveMode(message)) {
    return false;
  }

  std::string mode;
  if (!hardware_mode_client_->queryMode(mode, message)) {
    return false;
  }
  updateCurrentMode(mode);
  if (mode != "POSITION") {
    message = "Move actions require POSITION mode, current mode is " + mode;
    return false;
  }
  if (!joint_state_cache_->waitForFreshState(message)) {
    return false;
  }

  return true;
}

bool MotionServerNode::setMode(const std::string & requested_mode, std::string & message)
{
  const std::string mode = normalize_mode(requested_mode);
  if (mode == "FREE_DRIVE") {
    return enterFreeDriveMode(message);
  }
  if (!is_valid_mode(mode)) {
    message = "Unknown mode '" + requested_mode + "'. Expected POSITION, IDLE, DRAG, or FREE_DRIVE";
    return false;
  }
  if (!exitFreeDriveMode(message)) {
    return false;
  }

  return setHardwareMode(mode, message);
}

bool MotionServerNode::setHardwareMode(const std::string & mode, std::string & message)
{
  if (mode == "POSITION" && !hold_trajectory_sender_->holdCurrentPosition(message)) {
    return false;
  }

  if (!hardware_mode_client_->setMode(mode, message)) {
    return false;
  }

  updateCurrentMode(mode);
  return true;
}

bool MotionServerNode::enterFreeDriveMode(std::string & message)
{
  if (position_servo_executor_->isActive()) {
    position_servo_executor_->stop();
  }
  if (moveit_servo_runtime_->isActive()) {
    moveit_servo_runtime_->stop();
  }
  moveit_executor_->stop();

  std::string hardware_mode;
  if (!hardware_mode_client_->queryMode(hardware_mode, message)) {
    return false;
  }
  updateCurrentMode(hardware_mode);
  if (hardware_mode != "POSITION" && !setHardwareMode("POSITION", message)) {
    return false;
  }
  if (!joint_state_cache_->waitForFreshState(message)) {
    return false;
  }
  if (!switchControllers(freedrive_controller_name_, trajectory_controller_name_, message)) {
    return false;
  }

  updateCurrentMode("FREE_DRIVE");
  message = "Mode set to FREE_DRIVE";
  return true;
}

bool MotionServerNode::exitFreeDriveMode(std::string & message)
{
  const auto freedrive_state = controllerState(freedrive_controller_name_, message);
  if (!freedrive_state.has_value()) {
    return message.find("is not loaded") != std::string::npos;
  }
  if (*freedrive_state == "active") {
    return switchControllers(trajectory_controller_name_, freedrive_controller_name_, message);
  }
  return true;
}

bool MotionServerNode::switchControllers(
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

  if (!switch_controller_client_->wait_for_service(kControllerServiceWaitTimeout)) {
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
  if (future.wait_for(kControllerServiceCallTimeout) != std::future_status::ready) {
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

std::optional<std::string> MotionServerNode::controllerState(
  const std::string & controller_name,
  std::string & message)
{
  if (!list_controllers_client_->wait_for_service(kControllerServiceWaitTimeout)) {
    message = "/controller_manager/list_controllers service is not available";
    return std::nullopt;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = list_controllers_client_->async_send_request(request);
  if (future.wait_for(kControllerServiceCallTimeout) != std::future_status::ready) {
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

void MotionServerNode::updateCurrentMode(const std::string & mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_mode_ = mode;
}

std::string MotionServerNode::activeTaskSnapshot()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return active_task_;
}

void MotionServerNode::publishMoveJFeedback(
  const std::shared_ptr<GoalHandleMoveJ> & goal_handle,
  const std::string & state)
{
  auto feedback = std::make_shared<MoveJ::Feedback>();
  feedback->state = state;
  goal_handle->publish_feedback(feedback);
}

void MotionServerNode::publishMoveLFeedback(
  const std::shared_ptr<GoalHandleMoveL> & goal_handle,
  const std::string & state)
{
  auto feedback = std::make_shared<MoveL::Feedback>();
  feedback->state = state;
  goal_handle->publish_feedback(feedback);
}

void MotionServerNode::publishMoveNamedStateFeedback(
  const std::shared_ptr<GoalHandleMoveNamedState> & goal_handle,
  const std::string & state)
{
  auto feedback = std::make_shared<MoveNamedState::Feedback>();
  feedback->state = state;
  goal_handle->publish_feedback(feedback);
}

void MotionServerNode::handleSetMode(
  const std::shared_ptr<easyarm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<easyarm_interfaces::srv::SetMode::Response> response)
{
  if (!claimTask("SetMode", response->message)) {
    response->success = false;
    return;
  }
  response->success = setMode(request->mode, response->message);
  releaseTask();
}

void MotionServerNode::handleStop(
  const std::shared_ptr<easyarm_interfaces::srv::Stop::Request>,
  std::shared_ptr<easyarm_interfaces::srv::Stop::Response> response)
{
  stop_requested_.store(true);
  moveit_executor_->stop();
  if (position_servo_executor_->isActive()) {
    position_servo_executor_->stop();
  }
  if (moveit_servo_runtime_->isActive()) {
    moveit_servo_runtime_->stop();
    const auto active_task = activeTaskSnapshot();
    if (!moveit_servo_runtime_->isActive() &&
      (active_task.rfind("Speed", 0) == 0 || active_task.rfind("Servo", 0) == 0))
    {
      releaseTask();
    }
  }
  response->success = true;
  response->message = "Stop requested";
}

void MotionServerNode::handleGetState(
  const std::shared_ptr<easyarm_interfaces::srv::GetState::Request>,
  std::shared_ptr<easyarm_interfaces::srv::GetState::Response> response)
{
  std::string controller_message;
  const auto freedrive_state = controllerState(freedrive_controller_name_, controller_message);
  const bool freedrive_active = freedrive_state.has_value() && *freedrive_state == "active";

  std::string mode;
  std::string message;
  if (!hardware_mode_client_->queryMode(mode, message)) {
    response->success = false;
    response->message = message;
    std::lock_guard<std::mutex> lock(state_mutex_);
    response->mode = current_mode_;
    response->busy = busy_.load();
    response->active_task = active_task_;
    return;
  }

  updateCurrentMode(freedrive_active ? "FREE_DRIVE" : mode);
  response->success = true;
  response->message = "OK";
  response->busy = busy_.load();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    response->mode = current_mode_;
    response->active_task = active_task_;
  }
}

void MotionServerNode::handleGetJoints(
  const std::shared_ptr<easyarm_interfaces::srv::GetJoints::Request>,
  std::shared_ptr<easyarm_interfaces::srv::GetJoints::Response> response)
{
  sensor_msgs::msg::JointState joint_state;
  if (!joint_state_cache_->snapshot(joint_state)) {
    response->success = false;
    response->message = "No /joint_states message received";
    return;
  }

  response->success = true;
  response->message = "OK";
  response->names = joint_state.name;
  response->positions = joint_state.position;
  response->velocities = joint_state.velocity;
  response->efforts = joint_state.effort;
}

void MotionServerNode::handleGetPose(
  const std::shared_ptr<easyarm_interfaces::srv::GetPose::Request> request,
  std::shared_ptr<easyarm_interfaces::srv::GetPose::Response> response)
{
  const std::string target_frame = request->target_frame.empty() ? context_.planning_frame : request->target_frame;
  const std::string source_frame = request->source_frame.empty() ? context_.ee_link : request->source_frame;

  response->success = moveit_executor_->getPose(target_frame, source_frame, response->pose, response->message);
}

void MotionServerNode::handleListNamedState(
  const std::shared_ptr<easyarm_interfaces::srv::ListNamedState::Request>,
  std::shared_ptr<easyarm_interfaces::srv::ListNamedState::Response> response)
{
  if (!moveit_executor_->isInitialized()) {
    response->success = false;
    response->message = "MoveGroupInterface is not initialized";
    return;
  }

  response->names = moveit_executor_->listNamedStates();
  response->joint_names.assign(context_.joint_names.begin(), context_.joint_names.end());
  response->positions.clear();
  response->positions.reserve(response->names.size() * context_.joint_names.size());
  for (const auto & name : response->names) {
    const auto values = moveit_executor_->getNamedStateValues(name);
    for (const auto & joint_name : context_.joint_names) {
      const auto value = values.find(joint_name);
      response->positions.push_back(value == values.end() ? 0.0 : value->second);
    }
  }
  response->success = true;
  response->message = "OK";
}

void MotionServerNode::handleSpeedJCommand(control_msgs::msg::JointJog::SharedPtr command)
{
  std::string message;
  if (!prepareServoCommand("SpeedJ", message)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Reject SpeedJ: %s", message.c_str());
    return;
  }
  moveit_servo_runtime_->forwardSpeedJ(*command);
}

void MotionServerNode::handleSpeedLCommand(geometry_msgs::msg::TwistStamped::SharedPtr command)
{
  std::string message;
  if (!prepareServoCommand("SpeedL", message)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Reject SpeedL: %s", message.c_str());
    return;
  }
  moveit_servo_runtime_->forwardSpeedL(*command);
}

void MotionServerNode::handleServoJCommand(trajectory_msgs::msg::JointTrajectory::SharedPtr command)
{
  std::string message;
  const bool runtime_was_active = moveit_servo_runtime_->isActive();
  if (!prepareServoCommand("ServoJ", message)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Reject ServoJ: %s", message.c_str());
    return;
  }
  if (!position_servo_executor_->acceptServoJTarget(*command, message)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Reject ServoJ: %s", message.c_str());
    if (!runtime_was_active && activeTaskSnapshot() == "ServoJ") {
      position_servo_executor_->stop();
      moveit_servo_runtime_->stop();
      releaseTask();
    }
  }
}

void MotionServerNode::handleServoLCommand(geometry_msgs::msg::PoseStamped::SharedPtr command)
{
  std::string message;
  const bool runtime_was_active = moveit_servo_runtime_->isActive();
  if (!prepareServoCommand("ServoL", message)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Reject ServoL: %s", message.c_str());
    return;
  }
  if (!position_servo_executor_->acceptServoLTarget(*command, message)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Reject ServoL: %s", message.c_str());
    if (!runtime_was_active && activeTaskSnapshot() == "ServoL") {
      position_servo_executor_->stop();
      moveit_servo_runtime_->stop();
      releaseTask();
    }
  }
}

void MotionServerNode::handleServoTimer()
{
  if (!moveit_servo_runtime_->isActive()) {
    return;
  }

  position_servo_executor_->update();
  moveit_servo_runtime_->update();
  const auto active_task = activeTaskSnapshot();
  if (!moveit_servo_runtime_->isActive() &&
    (active_task.rfind("Speed", 0) == 0 || active_task.rfind("Servo", 0) == 0))
  {
    releaseTask();
  }
}

bool MotionServerNode::prepareServoCommand(const std::string & task, std::string & message)
{
  if (moveit_servo_runtime_->isActive()) {
    const auto active_task = activeTaskSnapshot();
    if (active_task != task) {
      message = "SERVO runtime is busy with " + active_task;
      return false;
    }
    return true;
  }

  if (task.rfind("Servo", 0) == 0 && !position_servo_executor_->isInitialized()) {
    message = "PositionServoExecutor is not initialized";
    return false;
  }

  if (!claimTask(task, message)) {
    return false;
  }

  std::string mode;
  if (!hardware_mode_client_->queryMode(mode, message)) {
    releaseTask();
    return false;
  }
  updateCurrentMode(mode);
  if (mode != "POSITION") {
    message = task + " requires POSITION hardware mode, current mode is " + mode;
    releaseTask();
    return false;
  }
  if (!joint_state_cache_->waitForFreshState(message)) {
    releaseTask();
    return false;
  }

  moveit_executor_->stop();
  if (!moveit_servo_runtime_->enterServoRuntime(task, message)) {
    releaseTask();
    return false;
  }

  return true;
}

}  // namespace easyarm_motion_server
