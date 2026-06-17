#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <easyarm_interfaces/action/move_j.hpp>
#include <easyarm_interfaces/action/move_l.hpp>
#include <easyarm_interfaces/srv/get_joints.hpp>
#include <easyarm_interfaces/srv/get_pose.hpp>
#include <easyarm_interfaces/srv/get_state.hpp>
#include <easyarm_interfaces/srv/set_mode.hpp>
#include <easyarm_interfaces/srv/stop.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace easyarm_motion_server
{

namespace
{

using MoveJ = easyarm_interfaces::action::MoveJ;
using MoveL = easyarm_interfaces::action::MoveL;
using GoalHandleMoveJ = rclcpp_action::ServerGoalHandle<MoveJ>;
using GoalHandleMoveL = rclcpp_action::ServerGoalHandle<MoveL>;
using FollowJT = control_msgs::action::FollowJointTrajectory;

const std::array<std::string, 6> kJointNames = {
  "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"
};

std::string normalize_mode(std::string mode)
{
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return mode;
}

bool is_valid_mode(const std::string & mode)
{
  return mode == "POSITION" || mode == "IDLE" || mode == "DRAG";
}

bool is_success(const moveit::core::MoveItErrorCode & code)
{
  return code == moveit::core::MoveItErrorCode::SUCCESS;
}

geometry_msgs::msg::PoseStamped transform_to_pose(const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

}  // namespace

class EasyArmMotionServer : public rclcpp::Node
{
public:
  explicit EasyArmMotionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("easyarm_motion_server", options)
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    ee_link_ = declare_parameter<std::string>("ee_link", "Link6");
    planning_frame_ = declare_parameter<std::string>("planning_frame", "base_link");
    default_velocity_scale_ = declare_parameter<double>("default_velocity_scale", 0.2);
    default_acceleration_scale_ = declare_parameter<double>("default_acceleration_scale", 0.2);
    movej_planner_id_ = declare_parameter<std::string>("movej_planner_id", "PTP");
    movel_planner_id_ = declare_parameter<std::string>("movel_planner_id", "LIN");
    planning_pipeline_id_ =
      declare_parameter<std::string>("planning_pipeline_id", "pilz_industrial_motion_planner");
    joint_state_wait_timeout_sec_ = declare_parameter<double>("joint_state_wait_timeout", 5.0);
    max_joint_state_age_sec_ = declare_parameter<double>("max_joint_state_age", 0.5);

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      std::bind(&EasyArmMotionServer::handle_joint_state, this, std::placeholders::_1));

    mode_client_ = create_client<rcl_interfaces::srv::SetParameters>(
      "/easyarm_hardware_control_mode/set_parameters",
      rmw_qos_profile_services_default,
      callback_group_);

    mode_get_client_ = create_client<rcl_interfaces::srv::GetParameters>(
      "/easyarm_hardware_control_mode/get_parameters",
      rmw_qos_profile_services_default,
      callback_group_);

    hold_client_ = rclcpp_action::create_client<FollowJT>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "arm_controller/follow_joint_trajectory",
      callback_group_);

    movej_server_ = rclcpp_action::create_server<MoveJ>(
      this,
      "/easyarm/movej",
      std::bind(&EasyArmMotionServer::handle_movej_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&EasyArmMotionServer::handle_movej_cancel, this, std::placeholders::_1),
      std::bind(&EasyArmMotionServer::handle_movej_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(),
      callback_group_);

    movel_server_ = rclcpp_action::create_server<MoveL>(
      this,
      "/easyarm/movel",
      std::bind(&EasyArmMotionServer::handle_movel_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&EasyArmMotionServer::handle_movel_cancel, this, std::placeholders::_1),
      std::bind(&EasyArmMotionServer::handle_movel_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(),
      callback_group_);

    set_mode_service_ = create_service<easyarm_interfaces::srv::SetMode>(
      "/easyarm/set_mode",
      std::bind(
        &EasyArmMotionServer::handle_set_mode,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    stop_service_ = create_service<easyarm_interfaces::srv::Stop>(
      "/easyarm/stop",
      std::bind(
        &EasyArmMotionServer::handle_stop,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    get_state_service_ = create_service<easyarm_interfaces::srv::GetState>(
      "/easyarm/get_state",
      std::bind(
        &EasyArmMotionServer::handle_get_state,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    get_joints_service_ = create_service<easyarm_interfaces::srv::GetJoints>(
      "/easyarm/get_joints",
      std::bind(
        &EasyArmMotionServer::handle_get_joints,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    get_pose_service_ = create_service<easyarm_interfaces::srv::GetPose>(
      "/easyarm/get_pose",
      std::bind(
        &EasyArmMotionServer::handle_get_pose,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "EasyArm motion server ready: group=%s, ee_link=%s, frame=%s, pipeline=%s",
      planning_group_.c_str(),
      ee_link_.c_str(),
      planning_frame_.c_str(),
      planning_pipeline_id_.c_str());
  }

  void initialize_move_group()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
    move_group_->setEndEffectorLink(ee_link_);
    move_group_->setPoseReferenceFrame(planning_frame_);
    RCLCPP_INFO(get_logger(), "MoveGroupInterface initialized for group '%s'", planning_group_.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_movej_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveJ::Goal> goal)
  {
    if (goal->joints.size() != 6) {
      RCLCPP_WARN(get_logger(), "Reject MoveJ: expected 6 joints, got %zu", goal->joints.size());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (busy_.load()) {
      const auto task = active_task_snapshot();
      RCLCPP_WARN(get_logger(), "Reject MoveJ: server is busy with %s", task.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_movej_cancel(const std::shared_ptr<GoalHandleMoveJ>)
  {
    stop_requested_.store(true);
    stop_move_group();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_movej_accepted(const std::shared_ptr<GoalHandleMoveJ> goal_handle)
  {
    std::thread{std::bind(&EasyArmMotionServer::execute_movej, this, goal_handle)}.detach();
  }

  rclcpp_action::GoalResponse handle_movel_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveL::Goal>)
  {
    if (busy_.load()) {
      const auto task = active_task_snapshot();
      RCLCPP_WARN(get_logger(), "Reject MoveL: server is busy with %s", task.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_movel_cancel(const std::shared_ptr<GoalHandleMoveL>)
  {
    stop_requested_.store(true);
    stop_move_group();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_movel_accepted(const std::shared_ptr<GoalHandleMoveL> goal_handle)
  {
    std::thread{std::bind(&EasyArmMotionServer::execute_movel, this, goal_handle)}.detach();
  }

  void execute_movej(const std::shared_ptr<GoalHandleMoveJ> goal_handle)
  {
    auto result = std::make_shared<MoveJ::Result>();
    const auto goal = goal_handle->get_goal();

    if (!claim_task("MoveJ", result->message)) {
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    publish_movej_feedback(goal_handle, "preparing");

    bool success = false;
    std::string message;
    try {
      success = run_movej(*goal, goal_handle, message);
    } catch (const std::exception & exception) {
      message = exception.what();
      success = false;
    }

    if (goal_handle->is_canceling() || stop_requested_.load()) {
      result->success = false;
      result->message = "MoveJ canceled";
      goal_handle->canceled(result);
      release_task();
      return;
    }

    result->success = success;
    result->message = message;
    if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
    release_task();
  }

  void execute_movel(const std::shared_ptr<GoalHandleMoveL> goal_handle)
  {
    auto result = std::make_shared<MoveL::Result>();
    const auto goal = goal_handle->get_goal();

    if (!claim_task("MoveL", result->message)) {
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    publish_movel_feedback(goal_handle, "preparing");

    bool success = false;
    std::string message;
    try {
      success = run_movel(*goal, goal_handle, message);
    } catch (const std::exception & exception) {
      message = exception.what();
      success = false;
    }

    if (goal_handle->is_canceling() || stop_requested_.load()) {
      result->success = false;
      result->message = "MoveL canceled";
      goal_handle->canceled(result);
      release_task();
      return;
    }

    result->success = success;
    result->message = message;
    if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
    release_task();
  }

  bool run_movej(
    const MoveJ::Goal & goal,
    const std::shared_ptr<GoalHandleMoveJ> & goal_handle,
    std::string & message)
  {
    if (!prepare_motion(message)) {
      return false;
    }
    if (goal_handle->is_canceling()) {
      message = "MoveJ canceled before planning";
      return false;
    }

    publish_movej_feedback(goal_handle, "planning");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      configure_common_planning(goal.velocity_scale, goal.acceleration_scale);
      move_group_->setPlannerId(movej_planner_id_);
      if (!move_group_->setJointValueTarget(std::vector<double>(goal.joints.begin(), goal.joints.end()))) {
        message = "MoveJ joint target rejected by MoveIt";
        return false;
      }
      const auto plan_result = move_group_->plan(plan);
      if (!is_success(plan_result)) {
        message = "MoveJ planning failed";
        return false;
      }
    }

    if (!goal.execute) {
      message = "MoveJ planning succeeded";
      return true;
    }
    if (goal_handle->is_canceling()) {
      message = "MoveJ canceled before execution";
      return false;
    }

    publish_movej_feedback(goal_handle, "executing");
    const auto execute_result = move_group_->execute(plan);
    if (!is_success(execute_result)) {
      message = "MoveJ execution failed";
      return false;
    }

    message = "MoveJ execution succeeded";
    return true;
  }

  bool run_movel(
    const MoveL::Goal & goal,
    const std::shared_ptr<GoalHandleMoveL> & goal_handle,
    std::string & message)
  {
    if (!prepare_motion(message)) {
      return false;
    }
    if (goal_handle->is_canceling()) {
      message = "MoveL canceled before planning";
      return false;
    }

    publish_movel_feedback(goal_handle, "planning");

    auto target_pose = goal.target_pose;
    if (target_pose.header.frame_id.empty()) {
      target_pose.header.frame_id = planning_frame_;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      configure_common_planning(goal.velocity_scale, goal.acceleration_scale);
      move_group_->setPlannerId(movel_planner_id_);
      move_group_->setPoseReferenceFrame(target_pose.header.frame_id);
      if (!move_group_->setPoseTarget(target_pose, ee_link_)) {
        message = "MoveL pose target rejected by MoveIt";
        return false;
      }
      const auto plan_result = move_group_->plan(plan);
      move_group_->clearPoseTargets();
      move_group_->setPoseReferenceFrame(planning_frame_);
      if (!is_success(plan_result)) {
        message = "MoveL planning failed";
        return false;
      }
    }

    if (!goal.execute) {
      message = "MoveL planning succeeded";
      return true;
    }
    if (goal_handle->is_canceling()) {
      message = "MoveL canceled before execution";
      return false;
    }

    publish_movel_feedback(goal_handle, "executing");
    const auto execute_result = move_group_->execute(plan);
    if (!is_success(execute_result)) {
      message = "MoveL execution failed";
      return false;
    }

    message = "MoveL execution succeeded";
    return true;
  }

  bool claim_task(const std::string & task, std::string & message)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      message = "Motion server is busy with " + active_task_snapshot();
      return false;
    }
    stop_requested_.store(false);
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_task_ = task;
    return true;
  }

  void release_task()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_task_.clear();
    stop_requested_.store(false);
    busy_.store(false);
  }

  bool prepare_motion(std::string & message)
  {
    if (!move_group_) {
      message = "MoveGroupInterface is not initialized";
      return false;
    }

    std::string mode;
    if (!query_hardware_mode(mode, message)) {
      return false;
    }
    if (mode != "POSITION") {
      message = "MoveJ/MoveL require POSITION mode, current mode is " + mode;
      return false;
    }
    if (!wait_for_fresh_joint_state(message)) {
      return false;
    }

    return true;
  }

  void configure_common_planning(double velocity_scale, double acceleration_scale)
  {
    move_group_->setStartStateToCurrentState();
    move_group_->setPlanningPipelineId(planning_pipeline_id_);
    move_group_->setPoseReferenceFrame(planning_frame_);
    move_group_->setMaxVelocityScalingFactor(scale_or_default(velocity_scale, default_velocity_scale_));
    move_group_->setMaxAccelerationScalingFactor(scale_or_default(acceleration_scale, default_acceleration_scale_));
  }

  double scale_or_default(double value, double default_value) const
  {
    const double selected = value > 0.0 ? value : default_value;
    return std::clamp(selected, 0.01, 1.0);
  }

  bool set_hardware_mode(const std::string & requested_mode, std::string & message)
  {
    const std::string mode = normalize_mode(requested_mode);
    if (!is_valid_mode(mode)) {
      message = "Unknown mode '" + requested_mode + "'. Expected POSITION, IDLE, or DRAG";
      return false;
    }

    if (mode == "POSITION" && !hold_current_position(message)) {
      return false;
    }

    if (!mode_client_->wait_for_service(std::chrono::seconds(3))) {
      message = "Hardware control mode service is not available";
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(rclcpp::Parameter("controller_mode", mode).to_parameter_msg());

    auto future = mode_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      message = "Timeout setting hardware mode to " + mode;
      return false;
    }

    const auto response = future.get();
    for (const auto & result : response->results) {
      if (!result.successful) {
        message = result.reason.empty() ? "Failed to set hardware mode to " + mode : result.reason;
        return false;
      }
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_mode_ = mode;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    message = "Mode set to " + mode;
    return true;
  }

  bool query_hardware_mode(std::string & mode, std::string & message)
  {
    if (!mode_get_client_->wait_for_service(std::chrono::seconds(3))) {
      message = "Hardware control mode get_parameters service is not available";
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("controller_mode");

    auto future = mode_get_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      message = "Timeout reading hardware controller_mode";
      return false;
    }

    const auto response = future.get();
    if (response->values.empty() ||
      response->values.front().type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    {
      message = "Hardware controller_mode parameter is not a string";
      return false;
    }

    mode = normalize_mode(response->values.front().string_value);
    if (!is_valid_mode(mode)) {
      message = "Hardware controller_mode has unknown value '" + response->values.front().string_value + "'";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_mode_ = mode;
    }
    return true;
  }

  bool wait_for_current_joint_positions(std::vector<double> & positions, std::string & message)
  {
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (read_current_joint_positions(positions)) {
        return true;
      }

      if (std::chrono::steady_clock::now() - start > std::chrono::seconds(3)) {
        message = "Timeout waiting for current /joint_states";
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    message = "ROS shutdown while waiting for current /joint_states";
    return false;
  }

  bool wait_for_fresh_joint_state(std::string & message)
  {
    const auto timeout = std::chrono::duration<double>(joint_state_wait_timeout_sec_);
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (has_fresh_joint_state(message)) {
        return true;
      }

      if (std::chrono::steady_clock::now() - start > timeout) {
        if (message.empty()) {
          message = "Timeout waiting for fresh /joint_states";
        }
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    message = "ROS shutdown while waiting for fresh /joint_states";
    return false;
  }

  bool has_fresh_joint_state(std::string & message)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (!last_joint_state_) {
      message = "Waiting for /joint_states";
      return false;
    }

    const auto missing = missing_joint_names(*last_joint_state_);
    if (!missing.empty()) {
      message = "Waiting for /joint_states containing all arm joints; missing: " + missing;
      return false;
    }

    const auto stamp = rclcpp::Time(last_joint_state_->header.stamp);
    if (stamp.nanoseconds() == 0) {
      message = "Waiting for /joint_states with a valid timestamp";
      return false;
    }

    const auto age = now() - stamp;
    if (age < rclcpp::Duration(0, 0)) {
      message = "Waiting for /joint_states timestamp to synchronize with node clock";
      return false;
    }

    const auto max_age = rclcpp::Duration::from_seconds(max_joint_state_age_sec_);
    if (age > max_age) {
      std::ostringstream stream;
      stream << "Waiting for fresh /joint_states; latest sample is "
             << age.seconds() << "s old";
      message = stream.str();
      return false;
    }

    message.clear();
    return true;
  }

  std::string missing_joint_names(const sensor_msgs::msg::JointState & joint_state) const
  {
    std::string missing;
    for (const auto & joint_name : kJointNames) {
      if (std::find(joint_state.name.begin(), joint_state.name.end(), joint_name) == joint_state.name.end()) {
        if (!missing.empty()) {
          missing += ", ";
        }
        missing += joint_name;
      }
    }
    return missing;
  }

  bool read_current_joint_positions(std::vector<double> & positions)
  {
    positions.assign(kJointNames.size(), 0.0);
    std::vector<bool> received(kJointNames.size(), false);

    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (!last_joint_state_) {
      return false;
    }

    for (size_t i = 0; i < kJointNames.size(); ++i) {
      const auto it = std::find(
        last_joint_state_->name.begin(),
        last_joint_state_->name.end(),
        kJointNames[i]);
      if (it == last_joint_state_->name.end()) {
        continue;
      }

      const auto index = static_cast<size_t>(std::distance(last_joint_state_->name.begin(), it));
      if (index < last_joint_state_->position.size()) {
        positions[i] = last_joint_state_->position[index];
        received[i] = true;
      }
    }

    return std::all_of(received.begin(), received.end(), [](bool value) { return value; });
  }

  bool hold_current_position(std::string & message)
  {
    std::vector<double> positions;
    if (!wait_for_current_joint_positions(positions, message)) {
      return false;
    }

    if (!hold_client_->wait_for_action_server(std::chrono::seconds(3))) {
      message = "arm_controller/follow_joint_trajectory action server is not available";
      return false;
    }

    FollowJT::Goal goal;
    goal.trajectory.joint_names.assign(kJointNames.begin(), kJointNames.end());

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities.assign(kJointNames.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(0.2);
    goal.trajectory.points.push_back(point);

    RCLCPP_INFO(get_logger(), "Sending hold trajectory before switching to POSITION");
    for (size_t i = 0; i < positions.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  %s: %.4f rad", kJointNames[i].c_str(), positions[i]);
    }

    auto goal_handle_future = hold_client_->async_send_goal(goal);
    if (goal_handle_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      message = "Timeout waiting for hold trajectory acceptance";
      return false;
    }

    const auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      message = "Hold trajectory rejected by arm_controller";
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    return true;
  }

  void stop_move_group()
  {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    if (move_group_) {
      move_group_->stop();
    }
  }

  void publish_movej_feedback(const std::shared_ptr<GoalHandleMoveJ> & goal_handle, const std::string & state)
  {
    auto feedback = std::make_shared<MoveJ::Feedback>();
    feedback->state = state;
    goal_handle->publish_feedback(feedback);
  }

  void publish_movel_feedback(const std::shared_ptr<GoalHandleMoveL> & goal_handle, const std::string & state)
  {
    auto feedback = std::make_shared<MoveL::Feedback>();
    feedback->state = state;
    goal_handle->publish_feedback(feedback);
  }

  void handle_set_mode(
    const std::shared_ptr<easyarm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<easyarm_interfaces::srv::SetMode::Response> response)
  {
    if (!claim_task("SetMode", response->message)) {
      response->success = false;
      return;
    }
    response->success = set_hardware_mode(request->mode, response->message);
    release_task();
  }

  void handle_stop(
    const std::shared_ptr<easyarm_interfaces::srv::Stop::Request>,
    std::shared_ptr<easyarm_interfaces::srv::Stop::Response> response)
  {
    stop_requested_.store(true);
    stop_move_group();
    response->success = true;
    response->message = "Stop requested";
  }

  void handle_get_state(
    const std::shared_ptr<easyarm_interfaces::srv::GetState::Request>,
    std::shared_ptr<easyarm_interfaces::srv::GetState::Response> response)
  {
    response->success = true;
    response->message = "OK";
    response->busy = busy_.load();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      response->mode = current_mode_;
      response->active_task = active_task_;
    }
  }

  void handle_get_joints(
    const std::shared_ptr<easyarm_interfaces::srv::GetJoints::Request>,
    std::shared_ptr<easyarm_interfaces::srv::GetJoints::Response> response)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (!last_joint_state_) {
      response->success = false;
      response->message = "No /joint_states message received";
      return;
    }

    response->success = true;
    response->message = "OK";
    response->names = last_joint_state_->name;
    response->positions = last_joint_state_->position;
    response->velocities = last_joint_state_->velocity;
    response->efforts = last_joint_state_->effort;
  }

  void handle_get_pose(
    const std::shared_ptr<easyarm_interfaces::srv::GetPose::Request> request,
    std::shared_ptr<easyarm_interfaces::srv::GetPose::Response> response)
  {
    const std::string target_frame = request->target_frame.empty() ? planning_frame_ : request->target_frame;
    const std::string source_frame = request->source_frame.empty() ? ee_link_ : request->source_frame;

    try {
      const auto transform = tf_buffer_->lookupTransform(
        target_frame,
        source_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(0.2));
      response->success = true;
      response->message = "OK";
      response->pose = transform_to_pose(transform);
    } catch (const std::exception & exception) {
      response->success = false;
      response->message = exception.what();
    }
  }

  void handle_joint_state(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    last_joint_state_ = *msg;
  }

  std::string active_task_snapshot()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return active_task_;
  }

  std::string planning_group_;
  std::string ee_link_;
  std::string planning_frame_;
  double default_velocity_scale_{0.2};
  double default_acceleration_scale_{0.2};
  std::string movej_planner_id_{"PTP"};
  std::string movel_planner_id_{"LIN"};
  std::string planning_pipeline_id_{"pilz_industrial_motion_planner"};
  double joint_state_wait_timeout_sec_{5.0};
  double max_joint_state_age_sec_{0.5};

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::mutex move_group_mutex_;
  std::mutex state_mutex_;
  std::atomic_bool busy_{false};
  std::atomic_bool stop_requested_{false};
  std::string current_mode_{"UNKNOWN"};
  std::string active_task_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::mutex joint_state_mutex_;
  std::optional<sensor_msgs::msg::JointState> last_joint_state_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr mode_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr mode_get_client_;
  rclcpp_action::Client<FollowJT>::SharedPtr hold_client_;
  rclcpp_action::Server<MoveJ>::SharedPtr movej_server_;
  rclcpp_action::Server<MoveL>::SharedPtr movel_server_;
  rclcpp::Service<easyarm_interfaces::srv::SetMode>::SharedPtr set_mode_service_;
  rclcpp::Service<easyarm_interfaces::srv::Stop>::SharedPtr stop_service_;
  rclcpp::Service<easyarm_interfaces::srv::GetState>::SharedPtr get_state_service_;
  rclcpp::Service<easyarm_interfaces::srv::GetJoints>::SharedPtr get_joints_service_;
  rclcpp::Service<easyarm_interfaces::srv::GetPose>::SharedPtr get_pose_service_;
};

}  // namespace easyarm_motion_server

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<easyarm_motion_server::EasyArmMotionServer>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  node->initialize_move_group();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
