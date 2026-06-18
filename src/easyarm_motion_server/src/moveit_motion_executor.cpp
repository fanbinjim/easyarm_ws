#include "easyarm_motion_server/moveit_motion_executor.hpp"

#include <algorithm>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <tf2_ros/buffer.h>

namespace easyarm_motion_server
{

namespace
{

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

MoveItMotionExecutor::MoveItMotionExecutor(rclcpp::Node & node, const MotionContext & context)
: node_(node), context_(context)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void MoveItMotionExecutor::initialize(const rclcpp::Node::SharedPtr & node)
{
  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    node, context_.planning_group);
  move_group_->setEndEffectorLink(context_.ee_link);
  move_group_->setPoseReferenceFrame(context_.planning_frame);
  RCLCPP_INFO(node_.get_logger(), "MoveGroupInterface initialized for group '%s'", context_.planning_group.c_str());
}

bool MoveItMotionExecutor::isInitialized() const
{
  return static_cast<bool>(move_group_);
}

bool MoveItMotionExecutor::runMoveJ(
  const MoveJ::Goal & goal,
  const CancelCheck & is_canceling,
  const FeedbackPublisher & publish_feedback,
  std::string & message)
{
  if (is_canceling()) {
    message = "MoveJ canceled before planning";
    return false;
  }

  publish_feedback("planning");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    configureCommonPlanning(goal.velocity_scale, goal.acceleration_scale);
    move_group_->setPlannerId(context_.movej_planner_id);
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
  if (is_canceling()) {
    message = "MoveJ canceled before execution";
    return false;
  }

  publish_feedback("executing");
  const auto execute_result = move_group_->execute(plan);
  if (!is_success(execute_result)) {
    message = "MoveJ execution failed";
    return false;
  }

  message = "MoveJ execution succeeded";
  return true;
}

bool MoveItMotionExecutor::runMoveL(
  const MoveL::Goal & goal,
  const CancelCheck & is_canceling,
  const FeedbackPublisher & publish_feedback,
  std::string & message)
{
  if (is_canceling()) {
    message = "MoveL canceled before planning";
    return false;
  }

  publish_feedback("planning");

  auto target_pose = goal.target_pose;
  if (target_pose.header.frame_id.empty()) {
    target_pose.header.frame_id = context_.planning_frame;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    configureCommonPlanning(goal.velocity_scale, goal.acceleration_scale);
    move_group_->setPlannerId(context_.movel_planner_id);
    move_group_->setPoseReferenceFrame(target_pose.header.frame_id);
    if (!move_group_->setPoseTarget(target_pose, context_.ee_link)) {
      message = "MoveL pose target rejected by MoveIt";
      return false;
    }
    const auto plan_result = move_group_->plan(plan);
    move_group_->clearPoseTargets();
    move_group_->setPoseReferenceFrame(context_.planning_frame);
    if (!is_success(plan_result)) {
      message = "MoveL planning failed";
      return false;
    }
  }

  if (!goal.execute) {
    message = "MoveL planning succeeded";
    return true;
  }
  if (is_canceling()) {
    message = "MoveL canceled before execution";
    return false;
  }

  publish_feedback("executing");
  const auto execute_result = move_group_->execute(plan);
  if (!is_success(execute_result)) {
    message = "MoveL execution failed";
    return false;
  }

  message = "MoveL execution succeeded";
  return true;
}

void MoveItMotionExecutor::stop()
{
  std::lock_guard<std::mutex> lock(move_group_mutex_);
  if (move_group_) {
    move_group_->stop();
  }
}

bool MoveItMotionExecutor::getPose(
  const std::string & target_frame,
  const std::string & source_frame,
  geometry_msgs::msg::PoseStamped & pose,
  std::string & message)
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      target_frame,
      source_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(0.2));
    pose = transform_to_pose(transform);
    message = "OK";
    return true;
  } catch (const std::exception & exception) {
    message = exception.what();
    return false;
  }
}

void MoveItMotionExecutor::configureCommonPlanning(double velocity_scale, double acceleration_scale)
{
  move_group_->setStartStateToCurrentState();
  move_group_->setPlanningPipelineId(context_.planning_pipeline_id);
  move_group_->setPoseReferenceFrame(context_.planning_frame);
  move_group_->setMaxVelocityScalingFactor(scaleOrDefault(velocity_scale, context_.default_velocity_scale));
  move_group_->setMaxAccelerationScalingFactor(
    scaleOrDefault(acceleration_scale, context_.default_acceleration_scale));
}

double MoveItMotionExecutor::scaleOrDefault(double value, double default_value) const
{
  const double selected = value > 0.0 ? value : default_value;
  return std::clamp(selected, 0.01, 1.0);
}

}  // namespace easyarm_motion_server
