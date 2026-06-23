#include "easyarm_motion_server/position_servo_executor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <control_msgs/msg/joint_jog.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/robot_state/robot_state.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace easyarm_motion_server
{
namespace
{
constexpr size_t kJointCount = 6;

double norm3(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}

Eigen::Quaterniond toQuaternion(const geometry_msgs::msg::Quaternion & message)
{
  Eigen::Quaterniond quaternion(message.w, message.x, message.y, message.z);
  if (quaternion.norm() <= std::numeric_limits<double>::epsilon()) {
    return Eigen::Quaterniond::Identity();
  }
  quaternion.normalize();
  return quaternion;
}

double angularDistance(
  const geometry_msgs::msg::Quaternion & target,
  const geometry_msgs::msg::Quaternion & reference)
{
  Eigen::Quaterniond q_error = toQuaternion(target) * toQuaternion(reference).inverse();
  if (q_error.w() < 0.0) {
    q_error.coeffs() *= -1.0;
  }
  return Eigen::AngleAxisd(q_error).angle();
}
}  // namespace

PositionServoExecutor::PositionServoExecutor(
  rclcpp::Node & node,
  const MotionContext & context,
  JointStateCache & joint_state_cache,
  MoveItServoRuntime & servo_runtime)
: node_(node),
  context_(context),
  joint_state_cache_(joint_state_cache),
  servo_runtime_(servo_runtime),
  last_target_time_(node_.get_clock()->now()),
  last_update_time_(node_.get_clock()->now())
{
  rate_hz_ = node_.declare_parameter<double>("position_servo_rate_hz", rate_hz_);
  command_timeout_sec_ =
    node_.declare_parameter<double>("position_servo_command_timeout_sec", command_timeout_sec_);
  servoj_kp_ = node_.declare_parameter<double>("servoj_kp", servoj_kp_);
  servoj_max_joint_velocity_rad_s_ =
    node_.declare_parameter<double>("servoj_max_joint_velocity_rad_s", servoj_max_joint_velocity_rad_s_);
  servoj_max_joint_acceleration_rad_s2_ =
    node_.declare_parameter<double>("servoj_max_joint_acceleration_rad_s2", servoj_max_joint_acceleration_rad_s2_);
  servoj_target_jump_rad_ =
    node_.declare_parameter<double>("servoj_target_jump_rad", servoj_target_jump_rad_);
  servoj_goal_tolerance_rad_ =
    node_.declare_parameter<double>("servoj_goal_tolerance_rad", servoj_goal_tolerance_rad_);
  servol_linear_kp_ = node_.declare_parameter<double>("servol_linear_kp", servol_linear_kp_);
  servol_angular_kp_ = node_.declare_parameter<double>("servol_angular_kp", servol_angular_kp_);
  servol_max_linear_velocity_m_s_ =
    node_.declare_parameter<double>("servol_max_linear_velocity_m_s", servol_max_linear_velocity_m_s_);
  servol_max_linear_acceleration_m_s2_ =
    node_.declare_parameter<double>("servol_max_linear_acceleration_m_s2", servol_max_linear_acceleration_m_s2_);
  servol_max_angular_velocity_rad_s_ =
    node_.declare_parameter<double>("servol_max_angular_velocity_rad_s", servol_max_angular_velocity_rad_s_);
  servol_max_angular_acceleration_rad_s2_ =
    node_.declare_parameter<double>("servol_max_angular_acceleration_rad_s2", servol_max_angular_acceleration_rad_s2_);
  servol_target_jump_m_ =
    node_.declare_parameter<double>("servol_target_jump_m", servol_target_jump_m_);
  servol_target_jump_rad_ =
    node_.declare_parameter<double>("servol_target_jump_rad", servol_target_jump_rad_);
  servol_position_tolerance_m_ =
    node_.declare_parameter<double>("servol_position_tolerance_m", servol_position_tolerance_m_);
  servol_orientation_tolerance_rad_ =
    node_.declare_parameter<double>("servol_orientation_tolerance_rad", servol_orientation_tolerance_rad_);
}

void PositionServoExecutor::initialize(const rclcpp::Node::SharedPtr & node)
{
  robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(
    node,
    "robot_description",
    true);
  robot_model_ = robot_model_loader_->getModel();
  if (!robot_model_) {
    RCLCPP_ERROR(node_.get_logger(), "Failed to load robot model for PositionServoExecutor");
    return;
  }
  RCLCPP_INFO(node_.get_logger(), "PositionServoExecutor initialized for group '%s'", context_.planning_group.c_str());
}

bool PositionServoExecutor::isInitialized() const
{
  return static_cast<bool>(robot_model_);
}

bool PositionServoExecutor::isActive() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return command_type_ != CommandType::None;
}

std::string PositionServoExecutor::activeTask() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (command_type_ == CommandType::ServoJ) {
    return "ServoJ";
  }
  if (command_type_ == CommandType::ServoL) {
    return "ServoL";
  }
  return "";
}

bool PositionServoExecutor::acceptServoJTarget(
  const trajectory_msgs::msg::JointTrajectory & command,
  std::string & message)
{
  ServoJTarget target;
  if (!parseServoJTarget(command, target, message)) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_type_ == CommandType::ServoL) {
      message = "Position servo is busy with ServoL";
      return false;
    }

    std::array<double, kJointCount> reference_joints{};
    if (command_type_ == CommandType::ServoJ) {
      reference_joints = servoj_target_.joints;
    } else {
      std::vector<double> current_positions;
      if (!currentJointPositions(current_positions, message)) {
        return false;
      }
      std::copy(current_positions.begin(), current_positions.end(), reference_joints.begin());
    }

    for (size_t i = 0; i < kJointCount; ++i) {
      if (std::abs(target.joints[i] - reference_joints[i]) > servoj_target_jump_rad_) {
        message = "ServoJ target jump exceeds servoj_target_jump_rad";
        return false;
      }
    }

    servoj_target_ = target;
    command_type_ = CommandType::ServoJ;
    last_target_time_ = node_.get_clock()->now();
    if (last_update_time_.nanoseconds() == 0) {
      last_update_time_ = last_target_time_;
    }
  }

  message = "ServoJ target accepted";
  return true;
}

bool PositionServoExecutor::acceptServoLTarget(
  const geometry_msgs::msg::PoseStamped & command,
  std::string & message)
{
  ServoLTarget target;
  if (!parseServoLTarget(command, target, message)) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_type_ == CommandType::ServoJ) {
      message = "Position servo is busy with ServoJ";
      return false;
    }

    geometry_msgs::msg::PoseStamped reference_pose;
    if (command_type_ == CommandType::ServoL) {
      reference_pose = servol_target_.pose;
    } else {
      std::vector<double> current_positions;
      if (!currentJointPositions(current_positions, message) ||
        !currentEndEffectorPose(current_positions, reference_pose, message))
      {
        return false;
      }
    }

    const auto linear_jump = norm3(
      target.pose.pose.position.x - reference_pose.pose.position.x,
      target.pose.pose.position.y - reference_pose.pose.position.y,
      target.pose.pose.position.z - reference_pose.pose.position.z);
    const auto angular_jump = angularDistance(target.pose.pose.orientation, reference_pose.pose.orientation);
    if (linear_jump > servol_target_jump_m_ || angular_jump > servol_target_jump_rad_) {
      message = "ServoL target jump exceeds servol target jump limits";
      return false;
    }

    servol_target_ = target;
    command_type_ = CommandType::ServoL;
    last_target_time_ = node_.get_clock()->now();
    if (last_update_time_.nanoseconds() == 0) {
      last_update_time_ = last_target_time_;
    }
  }

  message = "ServoL target accepted";
  return true;
}

void PositionServoExecutor::update()
{
  const auto now = node_.get_clock()->now();
  CommandType command_type;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    command_type = command_type_;
    if (command_type == CommandType::None) {
      return;
    }
    if ((now - last_target_time_).seconds() > command_timeout_sec_) {
      publishZeroForActiveCommand();
      resetLocked();
      std::string message;
      servo_runtime_.exitServoRuntime(message);
      RCLCPP_INFO(node_.get_logger(), "Position servo command timeout");
      return;
    }
  }

  if (command_type == CommandType::ServoJ) {
    updateServoJ(now);
  } else if (command_type == CommandType::ServoL) {
    updateServoL(now);
  }
}

void PositionServoExecutor::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  publishZeroForActiveCommand();
  resetLocked();
}

bool PositionServoExecutor::parseServoJTarget(
  const trajectory_msgs::msg::JointTrajectory & command,
  ServoJTarget & target,
  std::string & message) const
{
  if (command.joint_names.empty()) {
    message = "ServoJ requires joint_names";
    return false;
  }
  if (command.points.empty()) {
    message = "ServoJ requires points[0]";
    return false;
  }
  if (command.points.front().positions.size() != command.joint_names.size()) {
    message = "ServoJ points[0].positions size must match joint_names size";
    return false;
  }

  std::array<bool, kJointCount> received{};
  for (size_t i = 0; i < command.joint_names.size(); ++i) {
    const auto it = std::find(context_.joint_names.begin(), context_.joint_names.end(), command.joint_names[i]);
    if (it == context_.joint_names.end()) {
      continue;
    }
    const auto index = static_cast<size_t>(std::distance(context_.joint_names.begin(), it));
    target.joints[index] = command.points.front().positions[i];
    received[index] = true;
  }

  if (!std::all_of(received.begin(), received.end(), [](bool value) { return value; })) {
    message = "ServoJ requires all arm joints Joint1-Joint6";
    return false;
  }
  return true;
}

bool PositionServoExecutor::parseServoLTarget(
  const geometry_msgs::msg::PoseStamped & command,
  ServoLTarget & target,
  std::string & message) const
{
  target.pose = command;
  if (target.pose.header.frame_id.empty()) {
    target.pose.header.frame_id = context_.planning_frame;
  }
  if (target.pose.header.frame_id != context_.planning_frame) {
    message = "ServoL target frame must be empty or " + context_.planning_frame;
    return false;
  }
  return true;
}

bool PositionServoExecutor::updateServoJ(const rclcpp::Time & now)
{
  std::array<double, kJointCount> target;
  std::array<double, kJointCount> previous_velocity;
  rclcpp::Time previous_update;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target = servoj_target_.joints;
    previous_velocity = last_joint_velocity_;
    previous_update = last_update_time_;
  }

  std::vector<double> current_positions;
  std::string message;
  if (!currentJointPositions(current_positions, message)) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000, "ServoJ skipped: %s", message.c_str());
    return false;
  }

  const auto dt = std::max(1.0 / rate_hz_, (now - previous_update).seconds());
  const auto max_delta = servoj_max_joint_acceleration_rad_s2_ * dt;
  control_msgs::msg::JointJog jog;
  jog.header.stamp = now;
  jog.header.frame_id = context_.planning_frame;
  jog.joint_names.assign(context_.joint_names.begin(), context_.joint_names.end());
  jog.velocities.resize(kJointCount, 0.0);

  bool reached = true;
  std::array<double, kJointCount> next_velocity{};
  for (size_t i = 0; i < kJointCount; ++i) {
    const auto error = target[i] - current_positions[i];
    if (std::abs(error) > servoj_goal_tolerance_rad_) {
      reached = false;
    }
    const auto raw_velocity = clampAbs(servoj_kp_ * error, servoj_max_joint_velocity_rad_s_);
    const auto limited_velocity = limitRate(raw_velocity, previous_velocity[i], max_delta);
    next_velocity[i] = reached ? 0.0 : limited_velocity;
    jog.velocities[i] = next_velocity[i];
  }

  servo_runtime_.forwardSpeedJ(jog);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_joint_velocity_ = next_velocity;
    last_update_time_ = now;
  }
  return true;
}

bool PositionServoExecutor::updateServoL(const rclcpp::Time & now)
{
  ServoLTarget target;
  std::array<double, kJointCount> previous_twist;
  rclcpp::Time previous_update;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target = servol_target_;
    previous_twist = last_twist_;
    previous_update = last_update_time_;
  }

  std::vector<double> current_positions;
  geometry_msgs::msg::PoseStamped current_pose;
  std::string message;
  if (!currentJointPositions(current_positions, message) ||
    !currentEndEffectorPose(current_positions, current_pose, message))
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000, "ServoL skipped: %s", message.c_str());
    return false;
  }

  const auto dt = std::max(1.0 / rate_hz_, (now - previous_update).seconds());
  const auto max_linear_delta = servol_max_linear_acceleration_m_s2_ * dt;
  const auto max_angular_delta = servol_max_angular_acceleration_rad_s2_ * dt;
  std::array<double, kJointCount> raw_twist{
    servol_linear_kp_ * (target.pose.pose.position.x - current_pose.pose.position.x),
    servol_linear_kp_ * (target.pose.pose.position.y - current_pose.pose.position.y),
    servol_linear_kp_ * (target.pose.pose.position.z - current_pose.pose.position.z),
    0.0,
    0.0,
    0.0
  };

  const auto q_target = toQuaternion(target.pose.pose.orientation);
  const auto q_current = toQuaternion(current_pose.pose.orientation);
  Eigen::Quaterniond q_error = q_target * q_current.inverse();
  if (q_error.w() < 0.0) {
    q_error.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd angle_axis(q_error);
  raw_twist[3] = servol_angular_kp_ * angle_axis.axis().x() * angle_axis.angle();
  raw_twist[4] = servol_angular_kp_ * angle_axis.axis().y() * angle_axis.angle();
  raw_twist[5] = servol_angular_kp_ * angle_axis.axis().z() * angle_axis.angle();

  const auto position_error = norm3(
    target.pose.pose.position.x - current_pose.pose.position.x,
    target.pose.pose.position.y - current_pose.pose.position.y,
    target.pose.pose.position.z - current_pose.pose.position.z);
  const auto orientation_error = angle_axis.angle();
  const bool reached =
    position_error <= servol_position_tolerance_m_ &&
    orientation_error <= servol_orientation_tolerance_rad_;

  std::array<double, kJointCount> next_twist{};
  for (size_t i = 0; i < 3; ++i) {
    const auto clamped = clampAbs(raw_twist[i], servol_max_linear_velocity_m_s_);
    next_twist[i] = reached ? 0.0 : limitRate(clamped, previous_twist[i], max_linear_delta);
  }
  for (size_t i = 3; i < kJointCount; ++i) {
    const auto clamped = clampAbs(raw_twist[i], servol_max_angular_velocity_rad_s_);
    next_twist[i] = reached ? 0.0 : limitRate(clamped, previous_twist[i], max_angular_delta);
  }

  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = now;
  twist.header.frame_id = context_.planning_frame;
  twist.twist.linear.x = next_twist[0];
  twist.twist.linear.y = next_twist[1];
  twist.twist.linear.z = next_twist[2];
  twist.twist.angular.x = next_twist[3];
  twist.twist.angular.y = next_twist[4];
  twist.twist.angular.z = next_twist[5];

  servo_runtime_.forwardSpeedL(twist);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_twist_ = next_twist;
    last_update_time_ = now;
  }
  return true;
}

bool PositionServoExecutor::currentJointPositions(std::vector<double> & positions, std::string & message) const
{
  if (!joint_state_cache_.readCurrentJointPositions(positions)) {
    message = "No current joint positions available";
    return false;
  }
  return true;
}

bool PositionServoExecutor::currentEndEffectorPose(
  const std::vector<double> & joint_positions,
  geometry_msgs::msg::PoseStamped & pose,
  std::string & message) const
{
  if (!robot_model_) {
    message = "PositionServoExecutor robot model is not initialized";
    return false;
  }

  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  const auto * joint_model_group = robot_model_->getJointModelGroup(context_.planning_group);
  if (joint_model_group == nullptr) {
    message = "Planning group '" + context_.planning_group + "' not found";
    return false;
  }
  state.setJointGroupPositions(joint_model_group, joint_positions);
  state.updateLinkTransforms();

  const auto link_model = robot_model_->getLinkModel(context_.ee_link);
  if (link_model == nullptr) {
    message = "End-effector link '" + context_.ee_link + "' not found";
    return false;
  }

  const auto transform = state.getGlobalLinkTransform(link_model);
  pose.pose = tf2::toMsg(transform);
  pose.header.stamp = node_.get_clock()->now();
  pose.header.frame_id = context_.planning_frame;
  return true;
}

void PositionServoExecutor::publishZeroForActiveCommand()
{
  if (command_type_ == CommandType::ServoJ) {
    control_msgs::msg::JointJog jog;
    jog.header.stamp = node_.get_clock()->now();
    jog.header.frame_id = context_.planning_frame;
    jog.joint_names.assign(context_.joint_names.begin(), context_.joint_names.end());
    jog.velocities.assign(kJointCount, 0.0);
    servo_runtime_.forwardSpeedJ(jog);
  } else if (command_type_ == CommandType::ServoL) {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = node_.get_clock()->now();
    twist.header.frame_id = context_.planning_frame;
    servo_runtime_.forwardSpeedL(twist);
  }
}

void PositionServoExecutor::resetLocked()
{
  command_type_ = CommandType::None;
  last_joint_velocity_ = zeroJointCommand();
  last_twist_ = zeroJointCommand();
  last_update_time_ = node_.get_clock()->now();
}

double PositionServoExecutor::limitRate(double target, double previous, double max_delta) const
{
  return previous + std::clamp(target - previous, -max_delta, max_delta);
}

double PositionServoExecutor::clampAbs(double value, double limit) const
{
  return std::clamp(value, -std::abs(limit), std::abs(limit));
}

std::array<double, 6> PositionServoExecutor::zeroJointCommand() const
{
  return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

}  // namespace easyarm_motion_server
