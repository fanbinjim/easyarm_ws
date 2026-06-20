#include "easyarm_controller/dynamics_provider.hpp"

#include <chrono>
#include <thread>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace easyarm_controller
{
namespace
{
constexpr auto kRobotDescriptionTopic = "/robot_description";
constexpr auto kRobotDescriptionTimeout = std::chrono::seconds(3);
}  // namespace

bool DynamicsProvider::configure(
  const bool enable_feedforward,
  const double gravity_compensation_scale)
{
  if (gravity_compensation_scale < 0.0) {
    return false;
  }

  enable_feedforward_ = enable_feedforward;
  gravity_compensation_scale_ = gravity_compensation_scale;
  return true;
}

bool DynamicsProvider::initialize(
  const std::vector<std::string> & joint_names,
  const rclcpp::Logger & logger)
{
  joint_count_ = joint_names.size();
  robot_model_.reset();
  gravity_positions_.resize(0);
  gravity_torques_.resize(0);

  if (!enable_feedforward_) {
    return true;
  }

  std::string robot_description;
  std::string message;
  if (!waitForRobotDescription(robot_description, message)) {
    RCLCPP_ERROR(logger, "Failed to initialize dynamics model: %s", message.c_str());
    return false;
  }

  try {
    robot_model_ = std::make_unique<easyarm_dynamics::RobotModel>(
      easyarm_dynamics::RobotModel::fromUrdfXml(robot_description));
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger, "Failed to load dynamics model: %s", exception.what());
    return false;
  }

  const auto expected_size = static_cast<Eigen::Index>(joint_count_);
  if (robot_model_->nq() != expected_size || robot_model_->nv() != expected_size) {
    RCLCPP_ERROR(
      logger,
      "Dynamics model size mismatch: joints=%zu, nq=%ld, nv=%ld",
      joint_count_,
      static_cast<long>(robot_model_->nq()),
      static_cast<long>(robot_model_->nv()));
    robot_model_.reset();
    return false;
  }

  gravity_positions_.setZero(robot_model_->nq());
  gravity_torques_.setZero(robot_model_->nv());
  RCLCPP_INFO(logger, "Dynamics model loaded from /robot_description");
  return true;
}

bool DynamicsProvider::computeFeedforwardEffort(
  const std::vector<double> & positions,
  std::vector<double> & efforts,
  const rclcpp::Logger & logger)
{
  efforts.assign(joint_count_, 0.0);
  if (!enable_feedforward_) {
    return true;
  }
  if (!robot_model_) {
    RCLCPP_ERROR(logger, "Dynamics model is not initialized");
    return false;
  }
  if (positions.size() != joint_count_) {
    RCLCPP_ERROR(
      logger,
      "Cannot compute feedforward effort: expected %zu positions, got %zu",
      joint_count_,
      positions.size());
    return false;
  }

  for (size_t i = 0; i < positions.size(); ++i) {
    gravity_positions_[static_cast<Eigen::Index>(i)] = positions[i];
  }

  try {
    gravity_torques_ = robot_model_->gravity(gravity_positions_);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger, "Gravity computation failed: %s", exception.what());
    return false;
  }

  for (size_t i = 0; i < efforts.size(); ++i) {
    efforts[i] = gravity_torques_[static_cast<Eigen::Index>(i)] * gravity_compensation_scale_;
  }

  return true;
}

bool DynamicsProvider::waitForRobotDescription(
  std::string & robot_description,
  std::string & message) const
{
  auto node = std::make_shared<rclcpp::Node>("easyarm_controller_robot_description_loader");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  bool received = false;
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    kRobotDescriptionTopic,
    rclcpp::QoS(1).transient_local().reliable(),
    [&robot_description, &received](const std_msgs::msg::String::SharedPtr msg) {
      if (!msg->data.empty()) {
        robot_description = msg->data;
        received = true;
      }
    });

  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !received && std::chrono::steady_clock::now() - start < kRobotDescriptionTimeout) {
    executor.spin_some(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  executor.remove_node(node);
  subscription.reset();

  if (!received) {
    message = "timed out waiting for /robot_description";
    return false;
  }

  message = "OK";
  return true;
}

}  // namespace easyarm_controller
