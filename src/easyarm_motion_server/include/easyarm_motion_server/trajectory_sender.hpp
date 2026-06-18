#pragma once

#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "easyarm_motion_server/joint_state_cache.hpp"
#include "easyarm_motion_server/motion_context.hpp"

namespace easyarm_motion_server
{

class TrajectorySender
{
public:
  TrajectorySender(
    rclcpp::Node & node,
    const MotionContext & context,
    JointStateCache & joint_state_cache,
    const rclcpp::CallbackGroup::SharedPtr & callback_group);

  bool holdCurrentPosition(std::string & message);

private:
  using FollowJT = control_msgs::action::FollowJointTrajectory;

  rclcpp::Node & node_;
  const MotionContext & context_;
  JointStateCache & joint_state_cache_;
  rclcpp_action::Client<FollowJT>::SharedPtr hold_client_;
};

}  // namespace easyarm_motion_server
