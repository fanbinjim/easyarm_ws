#include "easyarm_motion_server/trajectory_sender.hpp"

#include <chrono>
#include <future>
#include <thread>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace easyarm_motion_server
{

TrajectorySender::TrajectorySender(
  rclcpp::Node & node,
  const MotionContext & context,
  JointStateCache & joint_state_cache,
  const rclcpp::CallbackGroup::SharedPtr & callback_group)
: node_(node), context_(context), joint_state_cache_(joint_state_cache)
{
  hold_client_ = rclcpp_action::create_client<FollowJT>(
    node_.get_node_base_interface(),
    node_.get_node_graph_interface(),
    node_.get_node_logging_interface(),
    node_.get_node_waitables_interface(),
    "arm_controller/follow_joint_trajectory",
    callback_group);
}

bool TrajectorySender::holdCurrentPosition(std::string & message)
{
  std::vector<double> positions;
  if (!joint_state_cache_.waitForCurrentJointPositions(positions, message)) {
    return false;
  }

  if (!hold_client_->wait_for_action_server(std::chrono::seconds(3))) {
    message = "arm_controller/follow_joint_trajectory action server is not available";
    return false;
  }

  FollowJT::Goal goal;
  goal.trajectory.joint_names.assign(context_.joint_names.begin(), context_.joint_names.end());

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.velocities.assign(context_.joint_names.size(), 0.0);
  point.time_from_start = rclcpp::Duration::from_seconds(0.2);
  goal.trajectory.points.push_back(point);

  RCLCPP_INFO(node_.get_logger(), "Sending hold trajectory before switching to POSITION");
  for (size_t i = 0; i < positions.size(); ++i) {
    RCLCPP_INFO(node_.get_logger(), "  %s: %.4f rad", context_.joint_names[i].c_str(), positions[i]);
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

}  // namespace easyarm_motion_server
