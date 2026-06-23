#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <easyarm_interfaces/action/move_j.hpp>
#include <easyarm_interfaces/action/move_l.hpp>
#include <easyarm_interfaces/action/move_named_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "easyarm_motion_server/motion_context.hpp"

namespace easyarm_motion_server
{

class MoveItMotionExecutor
{
public:
  using MoveJ = easyarm_interfaces::action::MoveJ;
  using MoveL = easyarm_interfaces::action::MoveL;
  using MoveNamedState = easyarm_interfaces::action::MoveNamedState;
  using CancelCheck = std::function<bool()>;
  using FeedbackPublisher = std::function<void(const std::string &)>;

  MoveItMotionExecutor(rclcpp::Node & node, const MotionContext & context);

  void initialize(const rclcpp::Node::SharedPtr & node);
  bool isInitialized() const;
  bool runMoveJ(
    const MoveJ::Goal & goal,
    const CancelCheck & is_canceling,
    const FeedbackPublisher & publish_feedback,
    std::string & message);
  bool runMoveL(
    const MoveL::Goal & goal,
    const CancelCheck & is_canceling,
    const FeedbackPublisher & publish_feedback,
    std::string & message);
  bool runMoveNamedState(
    const MoveNamedState::Goal & goal,
    const CancelCheck & is_canceling,
    const FeedbackPublisher & publish_feedback,
    std::string & message);
  std::vector<std::string> listNamedStates() const;
  std::map<std::string, double> getNamedStateValues(const std::string & name) const;
  void stop();
  bool getPose(
    const std::string & target_frame,
    const std::string & source_frame,
    geometry_msgs::msg::PoseStamped & pose,
    std::string & message);

private:
  void configureCommonPlanning(double velocity_scale, double acceleration_scale);
  double scaleOrDefault(double value, double default_value) const;

  rclcpp::Node & node_;
  const MotionContext & context_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  mutable std::mutex move_group_mutex_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace easyarm_motion_server
