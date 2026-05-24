#include <memory>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>(
    "move_to_ready",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

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
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
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

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();

  if (!execute_success) {
    return 1;
  }

  return 0;
}
