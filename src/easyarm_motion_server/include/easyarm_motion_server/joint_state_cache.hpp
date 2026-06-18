#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "easyarm_motion_server/motion_context.hpp"

namespace easyarm_motion_server
{

class JointStateCache
{
public:
  JointStateCache(rclcpp::Node & node, const MotionContext & context);

  bool waitForCurrentJointPositions(std::vector<double> & positions, std::string & message);
  bool waitForFreshState(std::string & message);
  bool readCurrentJointPositions(std::vector<double> & positions);
  bool snapshot(sensor_msgs::msg::JointState & joint_state) const;

private:
  bool hasFreshState(std::string & message);
  std::string missingJointNames(const sensor_msgs::msg::JointState & joint_state) const;
  void handleJointState(sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Node & node_;
  const MotionContext & context_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  mutable std::mutex joint_state_mutex_;
  std::optional<sensor_msgs::msg::JointState> last_joint_state_;
};

}  // namespace easyarm_motion_server
