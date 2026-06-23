#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "easyarm_motion_server/joint_state_cache.hpp"
#include "easyarm_motion_server/motion_context.hpp"
#include "easyarm_motion_server/moveit_servo_runtime.hpp"

namespace easyarm_motion_server
{

class PositionServoExecutor
{
public:
  PositionServoExecutor(
    rclcpp::Node & node,
    const MotionContext & context,
    JointStateCache & joint_state_cache,
    MoveItServoRuntime & servo_runtime);

  void initialize(const rclcpp::Node::SharedPtr & node);
  bool isInitialized() const;
  bool isActive() const;
  std::string activeTask() const;
  bool acceptServoJTarget(const trajectory_msgs::msg::JointTrajectory & command, std::string & message);
  bool acceptServoLTarget(const geometry_msgs::msg::PoseStamped & command, std::string & message);
  void update();
  void stop();

private:
  enum class CommandType
  {
    None,
    ServoJ,
    ServoL
  };

  struct ServoJTarget
  {
    std::array<double, 6> joints{};
  };

  struct ServoLTarget
  {
    geometry_msgs::msg::PoseStamped pose;
  };

  bool parseServoJTarget(
    const trajectory_msgs::msg::JointTrajectory & command,
    ServoJTarget & target,
    std::string & message) const;
  bool parseServoLTarget(
    const geometry_msgs::msg::PoseStamped & command,
    ServoLTarget & target,
    std::string & message) const;
  bool updateServoJ(const rclcpp::Time & now);
  bool updateServoL(const rclcpp::Time & now);
  bool currentJointPositions(std::vector<double> & positions, std::string & message) const;
  bool currentEndEffectorPose(
    const std::vector<double> & joint_positions,
    geometry_msgs::msg::PoseStamped & pose,
    std::string & message) const;
  void publishZeroForActiveCommand();
  void resetLocked();
  double limitRate(double target, double previous, double max_delta) const;
  double clampAbs(double value, double limit) const;
  std::array<double, 6> zeroJointCommand() const;

  rclcpp::Node & node_;
  const MotionContext & context_;
  JointStateCache & joint_state_cache_;
  MoveItServoRuntime & servo_runtime_;
  std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;

  mutable std::mutex mutex_;
  CommandType command_type_{CommandType::None};
  ServoJTarget servoj_target_;
  ServoLTarget servol_target_;
  rclcpp::Time last_target_time_;
  rclcpp::Time last_update_time_;
  std::array<double, 6> last_joint_velocity_{};
  std::array<double, 6> last_twist_{};

  double rate_hz_{200.0};
  double command_timeout_sec_{0.2};
  double servoj_kp_{30.0};
  double servoj_max_joint_velocity_rad_s_{30.0};
  double servoj_max_joint_acceleration_rad_s2_{120.0};
  double servoj_target_jump_rad_{0.5};
  double servoj_goal_tolerance_rad_{0.005};
  double servol_linear_kp_{20.0};
  double servol_angular_kp_{12.0};
  double servol_max_linear_velocity_m_s_{2.0};
  double servol_max_linear_acceleration_m_s2_{6.0};
  double servol_max_angular_velocity_rad_s_{10.0};
  double servol_max_angular_acceleration_rad_s2_{30.0};
  double servol_target_jump_m_{0.2};
  double servol_target_jump_rad_{1.0};
  double servol_position_tolerance_m_{0.001};
  double servol_orientation_tolerance_rad_{0.01};
};

}  // namespace easyarm_motion_server
