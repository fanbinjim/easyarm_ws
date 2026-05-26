#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "controller_mode_utils.hpp"

namespace
{

static const std::vector<std::string> kJointNames = {
  "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"
};

bool wait_for_joint_positions(
  rclcpp::Node & node, rclcpp::Logger logger,
  std::vector<double> & positions,
  std::chrono::seconds timeout = std::chrono::seconds(3))
{
  positions.assign(kJointNames.size(), 0.0);
  std::vector<bool> received(kJointNames.size(), false);

  bool got = false;
  auto sub = node.create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      for (size_t i = 0; i < kJointNames.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), kJointNames[i]);
        if (it != msg->name.end()) {
          auto idx = std::distance(msg->name.begin(), it);
          if (static_cast<size_t>(idx) < msg->position.size()) {
            positions[i] = msg->position[idx];
            received[i] = true;
          }
        }
      }
      got = std::all_of(received.begin(), received.end(), [](bool v) { return v; });
    });

  const auto start = std::chrono::steady_clock::now();

  while (!got && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (std::chrono::steady_clock::now() - start > timeout) {
      RCLCPP_ERROR(logger, "Timeout waiting for joint states");
      return false;
    }
  }

  return true;
}

bool hold_current_position(rclcpp::Node & node, rclcpp::Logger logger)
{
  using FollowJT = control_msgs::action::FollowJointTrajectory;

  std::vector<double> positions;
  if (!wait_for_joint_positions(node, logger, positions)) {
    return false;
  }

  auto action_client = rclcpp_action::create_client<FollowJT>(
    node.get_node_base_interface(),
    node.get_node_graph_interface(),
    node.get_node_logging_interface(),
    node.get_node_waitables_interface(),
    "arm_controller/follow_joint_trajectory");

  if (!action_client->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(logger, "arm_controller action server not available");
    return false;
  }

  auto goal = FollowJT::Goal();
  goal.trajectory.joint_names = kJointNames;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.velocities = std::vector<double>(kJointNames.size(), 0.0);
  point.time_from_start = rclcpp::Duration::from_seconds(0.2);
  goal.trajectory.points.push_back(point);

  RCLCPP_INFO(logger, "Sending hold trajectory to arm_controller");
  for (size_t i = 0; i < positions.size(); ++i) {
    RCLCPP_INFO(logger, "  %s: %.4f rad", kJointNames[i].c_str(), positions[i]);
  }

  auto send_goal_options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
  bool goal_accepted = false;
  bool goal_rejected = false;

  send_goal_options.goal_response_callback =
    [&](rclcpp_action::ClientGoalHandle<FollowJT>::SharedPtr goal_handle) {
      if (goal_handle) {
        goal_accepted = true;
        RCLCPP_INFO(logger, "Hold trajectory accepted by arm_controller");
      } else {
        goal_rejected = true;
        RCLCPP_ERROR(logger, "Hold trajectory rejected by arm_controller");
      }
    };

  send_goal_options.result_callback =
    [](const rclcpp_action::ClientGoalHandle<FollowJT>::WrappedResult &) {
    };

  action_client->async_send_goal(goal, send_goal_options);

  const auto start = std::chrono::steady_clock::now();

  while (!goal_accepted && !goal_rejected && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(3)) {
      RCLCPP_ERROR(logger, "Timeout waiting for goal acceptance");
      return false;
    }
  }

  if (goal_rejected) {
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  return true;
}

void shutdown_executor(
  rclcpp::executors::SingleThreadedExecutor & executor,
  std::thread & spinner)
{
  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>(
    "move_to_ready",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  RCLCPP_INFO(node->get_logger(), "Holding current position before switching to POSITION");
  if (!hold_current_position(*node, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed to hold current position");
    shutdown_executor(executor, spinner);
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Switching hardware to POSITION mode");
  if (set_controller_mode(*node, "POSITION")) {
    RCLCPP_INFO(node->get_logger(), "Switched to POSITION mode");
  } else {
    RCLCPP_WARN(node->get_logger(), "Failed to switch to POSITION mode");
  }

  static constexpr auto kPlanningGroup = "arm";
  static constexpr auto kNamedTarget = "ready";

  moveit::planning_interface::MoveGroupInterface move_group(node, kPlanningGroup);
  move_group.setStartStateToCurrentState();
  move_group.setNamedTarget(kNamedTarget);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  RCLCPP_INFO(
    node->get_logger(), "Planning motion for group '%s' to named target '%s'",
    kPlanningGroup, kNamedTarget);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool plan_success = static_cast<bool>(move_group.plan(plan));

  if (!plan_success) {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan motion to '%s'", kNamedTarget);
    shutdown_executor(executor, spinner);
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Plan successful. Executing motion...");
  const auto execute_result = move_group.execute(plan);
  const bool execute_success = execute_result == moveit::core::MoveItErrorCode::SUCCESS;

  if (execute_success) {
    RCLCPP_INFO(node->get_logger(), "Motion to '%s' completed", kNamedTarget);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to execute motion to '%s'", kNamedTarget);
  }

  shutdown_executor(executor, spinner);

  if (!execute_success) {
    return 1;
  }

  return 0;
}
