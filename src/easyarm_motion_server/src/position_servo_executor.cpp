#include "easyarm_motion_server/position_servo_executor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

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

geometry_msgs::msg::Quaternion toMessage(const Eigen::Quaterniond & quaternion)
{
  Eigen::Quaterniond normalized = quaternion;
  normalized.normalize();

  geometry_msgs::msg::Quaternion message;
  message.x = normalized.x();
  message.y = normalized.y();
  message.z = normalized.z();
  message.w = normalized.w();
  return message;
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

double positionDistance(
  const geometry_msgs::msg::PoseStamped & target,
  const geometry_msgs::msg::PoseStamped & reference)
{
  return norm3(
    target.pose.position.x - reference.pose.position.x,
    target.pose.position.y - reference.pose.position.y,
    target.pose.position.z - reference.pose.position.z);
}

geometry_msgs::msg::PoseStamped interpolatePose(
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & to,
  double linear_step,
  double angular_step)
{
  geometry_msgs::msg::PoseStamped result = from;
  result.header = to.header;

  const double dx = to.pose.position.x - from.pose.position.x;
  const double dy = to.pose.position.y - from.pose.position.y;
  const double dz = to.pose.position.z - from.pose.position.z;
  const double distance = norm3(dx, dy, dz);
  const double linear_ratio = distance <= std::numeric_limits<double>::epsilon() ?
    1.0 : std::min(1.0, std::max(0.0, linear_step) / distance);
  result.pose.position.x = from.pose.position.x + dx * linear_ratio;
  result.pose.position.y = from.pose.position.y + dy * linear_ratio;
  result.pose.position.z = from.pose.position.z + dz * linear_ratio;

  Eigen::Quaterniond q_from = toQuaternion(from.pose.orientation);
  Eigen::Quaterniond q_to = toQuaternion(to.pose.orientation);
  if (q_from.dot(q_to) < 0.0) {
    q_to.coeffs() *= -1.0;
  }
  const double angle = Eigen::AngleAxisd(q_to * q_from.inverse()).angle();
  const double angular_ratio = angle <= std::numeric_limits<double>::epsilon() ?
    1.0 : std::min(1.0, std::max(0.0, angular_step) / angle);
  result.pose.orientation = toMessage(q_from.slerp(angular_ratio, q_to));
  return result;
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
  servol_position_tolerance_exit_m_ =
    node_.declare_parameter<double>("servol_position_tolerance_exit_m", servol_position_tolerance_exit_m_);
  servol_orientation_tolerance_exit_rad_ =
    node_.declare_parameter<double>("servol_orientation_tolerance_exit_rad", servol_orientation_tolerance_exit_rad_);
  servol_target_governor_enabled_ =
    node_.declare_parameter<bool>("servol_target_governor_enabled", servol_target_governor_enabled_);
  servol_max_follow_distance_m_ =
    node_.declare_parameter<double>("servol_max_follow_distance_m", servol_max_follow_distance_m_);
  servol_max_follow_angle_rad_ =
    node_.declare_parameter<double>("servol_max_follow_angle_rad", servol_max_follow_angle_rad_);
  servol_target_lost_distance_m_ =
    node_.declare_parameter<double>("servol_target_lost_distance_m", servol_target_lost_distance_m_);
  servol_target_lost_angle_rad_ =
    node_.declare_parameter<double>("servol_target_lost_angle_rad", servol_target_lost_angle_rad_);
  servol_reacquire_distance_m_ =
    node_.declare_parameter<double>("servol_reacquire_distance_m", servol_reacquire_distance_m_);
  servol_reacquire_angle_rad_ =
    node_.declare_parameter<double>("servol_reacquire_angle_rad", servol_reacquire_angle_rad_);
  servol_recover_linear_velocity_m_s_ =
    node_.declare_parameter<double>("servol_recover_linear_velocity_m_s", servol_recover_linear_velocity_m_s_);
  servol_recover_angular_velocity_rad_s_ =
    node_.declare_parameter<double>("servol_recover_angular_velocity_rad_s", servol_recover_angular_velocity_rad_s_);

  servol_external_target_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
    "/easyarm/servol_external_target_pose",
    rclcpp::QoS(10));
  servol_internal_target_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
    "/easyarm/servol_internal_target_pose",
    rclcpp::QoS(10));
  servol_debug_pub_ = node_.create_publisher<std_msgs::msg::String>(
    "/easyarm/servol_debug",
    rclcpp::QoS(10));
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
    if (!servol_internal_target_initialized_) {
      servol_internal_target_pose_ = reference_pose;
      servol_internal_target_initialized_ = true;
      servol_tracking_state_ = ServoLTrackingState::Tracking;
      servol_hold_active_ = false;
    }
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
  ServoLTarget external_target;
  geometry_msgs::msg::PoseStamped previous_internal_target;
  ServoLTrackingState tracking_state;
  std::array<double, kJointCount> previous_twist;
  rclcpp::Time previous_update;
  bool internal_target_initialized;
  bool hold_active;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    external_target = servol_target_;
    previous_internal_target = servol_internal_target_pose_;
    tracking_state = servol_tracking_state_;
    previous_twist = last_twist_;
    previous_update = last_update_time_;
    internal_target_initialized = servol_internal_target_initialized_;
    hold_active = servol_hold_active_;
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
  auto target = shapeServoLTarget(
    external_target.pose,
    current_pose,
    previous_internal_target,
    tracking_state,
    internal_target_initialized,
    dt);

  std::array<double, kJointCount> raw_twist{
    servol_linear_kp_ * (target.pose.position.x - current_pose.pose.position.x),
    servol_linear_kp_ * (target.pose.position.y - current_pose.pose.position.y),
    servol_linear_kp_ * (target.pose.position.z - current_pose.pose.position.z),
    0.0,
    0.0,
    0.0
  };

  const auto q_target = toQuaternion(target.pose.orientation);
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
    target.pose.position.x - current_pose.pose.position.x,
    target.pose.position.y - current_pose.pose.position.y,
    target.pose.position.z - current_pose.pose.position.z);
  const auto orientation_error = angle_axis.angle();
  if (hold_active) {
    hold_active =
      position_error <= servol_position_tolerance_exit_m_ &&
      orientation_error <= servol_orientation_tolerance_exit_rad_;
  } else {
    hold_active =
      position_error <= servol_position_tolerance_m_ &&
      orientation_error <= servol_orientation_tolerance_rad_;
  }

  if (tracking_state == ServoLTrackingState::Recovering &&
    position_error <= servol_position_tolerance_m_ &&
    orientation_error <= servol_orientation_tolerance_rad_)
  {
    tracking_state = ServoLTrackingState::Tracking;
  }

  std::array<double, kJointCount> next_twist{};
  for (size_t i = 0; i < 3; ++i) {
    const auto clamped = clampAbs(raw_twist[i], servol_max_linear_velocity_m_s_);
    next_twist[i] = hold_active ? 0.0 : limitRate(clamped, previous_twist[i], max_linear_delta);
  }
  for (size_t i = 3; i < kJointCount; ++i) {
    const auto clamped = clampAbs(raw_twist[i], servol_max_angular_velocity_rad_s_);
    next_twist[i] = hold_active ? 0.0 : limitRate(clamped, previous_twist[i], max_angular_delta);
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
  publishServoLDebugTargets(external_target.pose, target);
  publishServoLDebugState(
    tracking_state,
    external_target.pose,
    target,
    current_pose,
    position_error,
    orientation_error,
    hold_active);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    servol_internal_target_pose_ = target;
    servol_internal_target_initialized_ = true;
    servol_tracking_state_ = tracking_state;
    servol_hold_active_ = hold_active;
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

geometry_msgs::msg::PoseStamped PositionServoExecutor::shapeServoLTarget(
  const geometry_msgs::msg::PoseStamped & external_target,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & previous_internal_target,
  ServoLTrackingState & tracking_state,
  bool internal_target_initialized,
  double dt) const
{
  if (!servol_target_governor_enabled_ || !internal_target_initialized) {
    return external_target;
  }

  const auto external_distance = positionDistance(external_target, current_pose);
  const auto external_angle = angularDistance(external_target.pose.orientation, current_pose.pose.orientation);
  if (external_distance > servol_target_lost_distance_m_ ||
    external_angle > servol_target_lost_angle_rad_)
  {
    tracking_state = ServoLTrackingState::Recovering;
  } else if (
    tracking_state == ServoLTrackingState::Recovering &&
    external_distance <= servol_reacquire_distance_m_ &&
    external_angle <= servol_reacquire_angle_rad_)
  {
    tracking_state = ServoLTrackingState::Tracking;
  }

  const bool recovering = tracking_state == ServoLTrackingState::Recovering;
  const double linear_step = recovering ?
    servol_recover_linear_velocity_m_s_ * dt :
    servol_max_linear_velocity_m_s_ * dt;
  const double angular_step = recovering ?
    servol_recover_angular_velocity_rad_s_ * dt :
    servol_max_angular_velocity_rad_s_ * dt;

  auto shaped_target = interpolatePose(
    previous_internal_target,
    external_target,
    linear_step,
    angular_step);

  if (positionDistance(shaped_target, current_pose) > servol_max_follow_distance_m_ ||
    angularDistance(shaped_target.pose.orientation, current_pose.pose.orientation) > servol_max_follow_angle_rad_)
  {
    shaped_target = interpolatePose(
      current_pose,
      shaped_target,
      servol_max_follow_distance_m_,
      servol_max_follow_angle_rad_);
  }

  shaped_target.header.stamp = external_target.header.stamp;
  shaped_target.header.frame_id = context_.planning_frame;
  return shaped_target;
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

void PositionServoExecutor::publishServoLDebugTargets(
  const geometry_msgs::msg::PoseStamped & external_target,
  const geometry_msgs::msg::PoseStamped & internal_target) const
{
  if (servol_external_target_pub_) {
    auto message = external_target;
    message.header.stamp = node_.get_clock()->now();
    message.header.frame_id = context_.planning_frame;
    servol_external_target_pub_->publish(message);
  }
  if (servol_internal_target_pub_) {
    auto message = internal_target;
    message.header.stamp = node_.get_clock()->now();
    message.header.frame_id = context_.planning_frame;
    servol_internal_target_pub_->publish(message);
  }
}

void PositionServoExecutor::publishServoLDebugState(
  ServoLTrackingState tracking_state,
  const geometry_msgs::msg::PoseStamped & external_target,
  const geometry_msgs::msg::PoseStamped & internal_target,
  const geometry_msgs::msg::PoseStamped & current_pose,
  double position_error,
  double orientation_error,
  bool hold_active) const
{
  if (!servol_debug_pub_) {
    return;
  }

  std_msgs::msg::String message;
  std::ostringstream stream;
  stream
    << "state=" << servoLTrackingStateName(tracking_state)
    << " external_distance_m=" << positionDistance(external_target, current_pose)
    << " external_angle_rad=" << angularDistance(external_target.pose.orientation, current_pose.pose.orientation)
    << " internal_distance_m=" << positionDistance(internal_target, current_pose)
    << " internal_angle_rad=" << angularDistance(internal_target.pose.orientation, current_pose.pose.orientation)
    << " position_error_m=" << position_error
    << " orientation_error_rad=" << orientation_error
    << " hold=" << (hold_active ? "true" : "false");
  message.data = stream.str();
  servol_debug_pub_->publish(message);
}

std::string PositionServoExecutor::servoLTrackingStateName(ServoLTrackingState tracking_state) const
{
  if (tracking_state == ServoLTrackingState::Recovering) {
    return "RECOVERING";
  }
  return "TRACKING";
}

void PositionServoExecutor::resetLocked()
{
  command_type_ = CommandType::None;
  last_joint_velocity_ = zeroJointCommand();
  last_twist_ = zeroJointCommand();
  last_update_time_ = node_.get_clock()->now();
  servol_tracking_state_ = ServoLTrackingState::Tracking;
  servol_internal_target_initialized_ = false;
  servol_hold_active_ = false;
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
