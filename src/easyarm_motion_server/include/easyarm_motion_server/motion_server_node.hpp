#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <control_msgs/msg/joint_jog.hpp>
#include <easyarm_interfaces/action/move_j.hpp>
#include <easyarm_interfaces/action/move_l.hpp>
#include <easyarm_interfaces/srv/get_joints.hpp>
#include <easyarm_interfaces/srv/get_pose.hpp>
#include <easyarm_interfaces/srv/get_state.hpp>
#include <easyarm_interfaces/srv/set_mode.hpp>
#include <easyarm_interfaces/srv/stop.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "easyarm_motion_server/hardware_mode_client.hpp"
#include "easyarm_motion_server/joint_state_cache.hpp"
#include "easyarm_motion_server/motion_context.hpp"
#include "easyarm_motion_server/moveit_motion_executor.hpp"
#include "easyarm_motion_server/moveit_servo_runtime.hpp"
#include "easyarm_motion_server/hold_trajectory_sender.hpp"

namespace easyarm_motion_server
{

class MotionServerNode : public rclcpp::Node
{
public:
  explicit MotionServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void initializeMoveGroup();

private:
  using MoveJ = easyarm_interfaces::action::MoveJ;
  using MoveL = easyarm_interfaces::action::MoveL;
  using GoalHandleMoveJ = rclcpp_action::ServerGoalHandle<MoveJ>;
  using GoalHandleMoveL = rclcpp_action::ServerGoalHandle<MoveL>;

  rclcpp_action::GoalResponse handleMoveJGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveJ::Goal> goal);
  rclcpp_action::CancelResponse handleMoveJCancel(const std::shared_ptr<GoalHandleMoveJ> goal_handle);
  void handleMoveJAccepted(const std::shared_ptr<GoalHandleMoveJ> goal_handle);

  rclcpp_action::GoalResponse handleMoveLGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveL::Goal> goal);
  rclcpp_action::CancelResponse handleMoveLCancel(const std::shared_ptr<GoalHandleMoveL> goal_handle);
  void handleMoveLAccepted(const std::shared_ptr<GoalHandleMoveL> goal_handle);

  void executeMoveJ(const std::shared_ptr<GoalHandleMoveJ> goal_handle);
  void executeMoveL(const std::shared_ptr<GoalHandleMoveL> goal_handle);

  bool claimTask(const std::string & task, std::string & message);
  void releaseTask();
  bool prepareMotion(std::string & message);
  bool setHardwareMode(const std::string & requested_mode, std::string & message);
  void updateCurrentMode(const std::string & mode);
  std::string activeTaskSnapshot();

  void publishMoveJFeedback(const std::shared_ptr<GoalHandleMoveJ> & goal_handle, const std::string & state);
  void publishMoveLFeedback(const std::shared_ptr<GoalHandleMoveL> & goal_handle, const std::string & state);

  void handleSetMode(
    const std::shared_ptr<easyarm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<easyarm_interfaces::srv::SetMode::Response> response);
  void handleStop(
    const std::shared_ptr<easyarm_interfaces::srv::Stop::Request> request,
    std::shared_ptr<easyarm_interfaces::srv::Stop::Response> response);
  void handleGetState(
    const std::shared_ptr<easyarm_interfaces::srv::GetState::Request> request,
    std::shared_ptr<easyarm_interfaces::srv::GetState::Response> response);
  void handleGetJoints(
    const std::shared_ptr<easyarm_interfaces::srv::GetJoints::Request> request,
    std::shared_ptr<easyarm_interfaces::srv::GetJoints::Response> response);
  void handleGetPose(
    const std::shared_ptr<easyarm_interfaces::srv::GetPose::Request> request,
    std::shared_ptr<easyarm_interfaces::srv::GetPose::Response> response);
  void handleSpeedJCommand(control_msgs::msg::JointJog::SharedPtr command);
  void handleSpeedLCommand(geometry_msgs::msg::TwistStamped::SharedPtr command);
  void handleServoTimer();
  bool prepareServoCommand(const std::string & task, std::string & message);

  MotionContext context_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<JointStateCache> joint_state_cache_;
  std::unique_ptr<HardwareModeClient> hardware_mode_client_;
  std::unique_ptr<HoldTrajectorySender> hold_trajectory_sender_;
  std::unique_ptr<MoveItMotionExecutor> moveit_executor_;
  std::unique_ptr<MoveItServoRuntime> moveit_servo_runtime_;

  std::mutex state_mutex_;
  std::atomic_bool busy_{false};
  std::atomic_bool stop_requested_{false};
  std::string current_mode_{"UNKNOWN"};
  std::string active_task_;

  rclcpp_action::Server<MoveJ>::SharedPtr movej_server_;
  rclcpp_action::Server<MoveL>::SharedPtr movel_server_;
  rclcpp::Service<easyarm_interfaces::srv::SetMode>::SharedPtr set_mode_service_;
  rclcpp::Service<easyarm_interfaces::srv::Stop>::SharedPtr stop_service_;
  rclcpp::Service<easyarm_interfaces::srv::GetState>::SharedPtr get_state_service_;
  rclcpp::Service<easyarm_interfaces::srv::GetJoints>::SharedPtr get_joints_service_;
  rclcpp::Service<easyarm_interfaces::srv::GetPose>::SharedPtr get_pose_service_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr speedj_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr speedl_sub_;
  rclcpp::TimerBase::SharedPtr servo_timer_;
};

}  // namespace easyarm_motion_server
