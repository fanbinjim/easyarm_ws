#include "easyarm_motion_server/joint_state_cache.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <thread>

namespace easyarm_motion_server
{

JointStateCache::JointStateCache(rclcpp::Node & node, const MotionContext & context)
: node_(node), context_(context)
{
  joint_state_sub_ = node_.create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&JointStateCache::handleJointState, this, std::placeholders::_1));
}

bool JointStateCache::waitForCurrentJointPositions(std::vector<double> & positions, std::string & message)
{
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    if (readCurrentJointPositions(positions)) {
      return true;
    }

    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(3)) {
      message = "Timeout waiting for current /joint_states";
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  message = "ROS shutdown while waiting for current /joint_states";
  return false;
}

bool JointStateCache::waitForFreshState(std::string & message)
{
  const auto timeout = std::chrono::duration<double>(context_.joint_state_wait_timeout_sec);
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    if (hasFreshState(message)) {
      return true;
    }

    if (std::chrono::steady_clock::now() - start > timeout) {
      if (message.empty()) {
        message = "Timeout waiting for fresh /joint_states";
      }
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  message = "ROS shutdown while waiting for fresh /joint_states";
  return false;
}

bool JointStateCache::readCurrentJointPositions(std::vector<double> & positions)
{
  positions.assign(context_.joint_names.size(), 0.0);
  std::vector<bool> received(context_.joint_names.size(), false);

  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (!last_joint_state_) {
    return false;
  }

  for (size_t i = 0; i < context_.joint_names.size(); ++i) {
    const auto it = std::find(
      last_joint_state_->name.begin(),
      last_joint_state_->name.end(),
      context_.joint_names[i]);
    if (it == last_joint_state_->name.end()) {
      continue;
    }

    const auto index = static_cast<size_t>(std::distance(last_joint_state_->name.begin(), it));
    if (index < last_joint_state_->position.size()) {
      positions[i] = last_joint_state_->position[index];
      received[i] = true;
    }
  }

  return std::all_of(received.begin(), received.end(), [](bool value) { return value; });
}

bool JointStateCache::snapshot(sensor_msgs::msg::JointState & joint_state) const
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (!last_joint_state_) {
    return false;
  }

  joint_state = *last_joint_state_;
  return true;
}

bool JointStateCache::hasFreshState(std::string & message)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (!last_joint_state_) {
    message = "Waiting for /joint_states";
    return false;
  }

  const auto missing = missingJointNames(*last_joint_state_);
  if (!missing.empty()) {
    message = "Waiting for /joint_states containing all arm joints; missing: " + missing;
    return false;
  }

  const auto stamp = rclcpp::Time(last_joint_state_->header.stamp);
  if (stamp.nanoseconds() == 0) {
    message = "Waiting for /joint_states with a valid timestamp";
    return false;
  }

  const auto age = node_.now() - stamp;
  if (age < rclcpp::Duration(0, 0)) {
    message = "Waiting for /joint_states timestamp to synchronize with node clock";
    return false;
  }

  const auto max_age = rclcpp::Duration::from_seconds(context_.max_joint_state_age_sec);
  if (age > max_age) {
    std::ostringstream stream;
    stream << "Waiting for fresh /joint_states; latest sample is "
           << age.seconds() << "s old";
    message = stream.str();
    return false;
  }

  message.clear();
  return true;
}

std::string JointStateCache::missingJointNames(const sensor_msgs::msg::JointState & joint_state) const
{
  std::string missing;
  for (const auto & joint_name : context_.joint_names) {
    if (std::find(joint_state.name.begin(), joint_state.name.end(), joint_name) == joint_state.name.end()) {
      if (!missing.empty()) {
        missing += ", ";
      }
      missing += joint_name;
    }
  }
  return missing;
}

void JointStateCache::handleJointState(sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  last_joint_state_ = *msg;
}

}  // namespace easyarm_motion_server
