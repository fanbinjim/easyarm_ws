#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "controller_mode_utils.hpp"

namespace
{

static const std::vector<std::string> kJointNames = {
  "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"
};

bool wait_for_joint_positions(
  rclcpp::Node & node, rclcpp::Logger & logger,
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
      got = std::all_of(received.begin(), received.end(),
                        [](bool v) { return v; });
    });

  auto start = std::chrono::steady_clock::now();

  while (!got && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (std::chrono::steady_clock::now() - start > timeout) {
      RCLCPP_ERROR(logger, "Timeout waiting for joint states");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return true;
}

void hold_current_position(rclcpp::Node & node, rclcpp::Logger & logger)
{
  using FollowJT = control_msgs::action::FollowJointTrajectory;

  std::vector<double> positions;
  if (!wait_for_joint_positions(node, logger, positions)) {
    std::exit(1);
  }

  auto action_client = rclcpp_action::create_client<FollowJT>(
    node.get_node_base_interface(),
    node.get_node_graph_interface(),
    node.get_node_logging_interface(),
    node.get_node_waitables_interface(),
    "arm_controller/follow_joint_trajectory");

  if (!action_client->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(logger, "arm_controller action server not available");
    std::exit(1);
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

  send_goal_options.goal_response_callback =
    [&](typename rclcpp_action::ClientGoalHandle<FollowJT>::SharedPtr) {
      goal_accepted = true;
      RCLCPP_INFO(logger, "Hold trajectory accepted by arm_controller");
    };

  send_goal_options.result_callback =
    [](const typename rclcpp_action::ClientGoalHandle<FollowJT>::WrappedResult &) {
    };

  action_client->async_send_goal(goal, send_goal_options);

  auto start = std::chrono::steady_clock::now();

  while (!goal_accepted && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(3)) {
      RCLCPP_ERROR(logger, "Timeout waiting for goal acceptance");
      std::exit(1);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_FATAL(
      rclcpp::get_logger("switch_controller_mode"),
      "Usage: switch_controller_mode <IDLE|POSITION|DRAG>");
    return 1;
  }

  std::string mode = argv[1];
  std::transform(mode.begin(), mode.end(), mode.begin(),
                 [](unsigned char c) { return std::toupper(c); });

  if (mode != "IDLE" && mode != "POSITION" && mode != "DRAG") {
    RCLCPP_FATAL(
      rclcpp::get_logger("switch_controller_mode"),
      "Unknown mode '%s'. Expected IDLE, POSITION, or DRAG", argv[1]);
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>(
    "switch_controller_mode",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  if (mode == "POSITION") {
    RCLCPP_INFO(logger, "Holding current position before switching to POSITION");
    hold_current_position(*node, logger);
  }

  if (!set_controller_mode(*node, mode)) {
    RCLCPP_ERROR(logger, "Failed to set controller_mode to %s", mode.c_str());
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "controller_mode set to %s", mode.c_str());

  executor.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown();
  return 0;
}
