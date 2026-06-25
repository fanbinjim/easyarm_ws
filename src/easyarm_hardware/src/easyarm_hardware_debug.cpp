#include "easyarm_hardware/easyarm_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <string>
#include <sstream>

namespace easyarm_hardware
{

namespace
{

constexpr const char * kDefaultDebugLogDirectory = "/dev/shm";

int64_t steady_time_ns()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

uint8_t mode_to_u8(ControlMode mode)
{
  return static_cast<uint8_t>(mode);
}

uint8_t mode_to_u8(MotorControlMode mode)
{
  return mode == MotorControlMode::PositionCsp ? 1 : 0;
}

uint32_t clamp_to_u32(uint64_t value)
{
  return value > std::numeric_limits<uint32_t>::max() ?
         std::numeric_limits<uint32_t>::max() :
         static_cast<uint32_t>(value);
}

uint32_t elapsed_us(std::chrono::steady_clock::time_point start)
{
  const auto elapsed = std::chrono::steady_clock::now() - start;
  const auto us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
  return us < 0 ? 0u : clamp_to_u32(static_cast<uint64_t>(us));
}

std::string timestamped_debug_log_path()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
#if defined(_WIN32)
  localtime_s(&local_time, &time);
#else
  localtime_r(&time, &local_time);
#endif

  std::ostringstream name;
  name << "easyarm_log_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".bin";
  auto path = std::filesystem::path(kDefaultDebugLogDirectory) / name.str();
  for (int index = 1; std::filesystem::exists(path) && index < 1000; ++index) {
    std::ostringstream fallback_name;
    fallback_name << "easyarm_log_" << std::put_time(&local_time, "%Y%m%d_%H%M%S")
                  << "_" << std::setw(3) << std::setfill('0') << index << ".bin";
    path = std::filesystem::path(kDefaultDebugLogDirectory) / fallback_name.str();
  }
  return path.string();
}

}  // namespace

void EasyArmHardware::start_debug_logger()
{
  std::lock_guard<std::mutex> lock(debug_state_mutex_);
  if (!debug_logger_config_.enabled || debug_logger_.is_active()) {
    return;
  }

  std::string message;
  apply_debug_logger_enabled(true, message);
}

void EasyArmHardware::stop_debug_logger()
{
  std::lock_guard<std::mutex> lock(debug_state_mutex_);
  const bool was_active = debug_logger_.is_active();
  debug_logger_.stop();
  if (was_active) {
    RCLCPP_INFO(
      logger_,
      "Debug logger stopped: written=%lu, dropped=%lu, path=%s",
      static_cast<unsigned long>(debug_logger_.written_count()),
      static_cast<unsigned long>(debug_logger_.dropped_count()),
      debug_logger_config_.path.c_str());
  }
}

HardwareDebugSample EasyArmHardware::make_debug_sample(const rclcpp::Duration & period)
{
  HardwareDebugSample sample{};
  sample.seq = debug_sequence_++;
  sample.steady_time_ns = steady_time_ns();
  sample.period_s = period.seconds();
  sample.hardware_mode = mode_to_u8(control_mode_);
  sample.motor_mode = mode_to_u8(active_motor_mode_);
  sample.skipped_from_joint = kDebugNoSkippedJoint;
  sample.dropped_before = clamp_to_u32(debug_logger_.dropped_count());

  const size_t count = std::min(joint_configs_.size(), sample.joints.size());
  for (size_t i = 0; i < count; ++i) {
    const auto & config = joint_configs_[i];
    auto & joint = sample.joints[i];
    joint.state_position = hw_positions_[i];
    joint.state_velocity = hw_velocities_[i];
    joint.state_effort = hw_efforts_[i];
    joint.command_position = hw_commands_positions_[i];
    joint.command_velocity = hw_commands_velocities_[i];
    joint.command_effort = hw_commands_efforts_[i];
    joint.smoothed_position = smoothed_positions_[i];
    joint.smoothed_velocity = smoothed_velocities_[i];
    joint.motor_position = smoothed_positions_[i] * config.direction + config.position_offset;
    joint.motor_velocity = smoothed_velocities_[i] * config.direction;
    joint.motor_id = config.motor_id;
  }

  return sample;
}

void EasyArmHardware::fill_debug_joint_command(
  HardwareDebugSample & sample,
  size_t joint_index,
  double motor_position,
  double motor_velocity,
  double motor_torque,
  double kp,
  double kd,
  bool send_ok) const
{
  if (joint_index >= sample.joints.size() || joint_index >= joint_configs_.size()) {
    return;
  }

  auto & joint = sample.joints[joint_index];
  joint.state_position = hw_positions_[joint_index];
  joint.state_velocity = hw_velocities_[joint_index];
  joint.state_effort = hw_efforts_[joint_index];
  joint.command_position = hw_commands_positions_[joint_index];
  joint.command_velocity = hw_commands_velocities_[joint_index];
  joint.command_effort = hw_commands_efforts_[joint_index];
  joint.smoothed_position = smoothed_positions_[joint_index];
  joint.smoothed_velocity = smoothed_velocities_[joint_index];
  joint.motor_position = motor_position;
  joint.motor_velocity = motor_velocity;
  joint.motor_torque = motor_torque;
  joint.kp = kp;
  joint.kd = kd;
  joint.motor_id = joint_configs_[joint_index].motor_id;
  joint.send_ok = send_ok ? 1 : 0;
}

void EasyArmHardware::push_debug_sample(
  HardwareDebugSample & sample,
  std::chrono::steady_clock::time_point write_start,
  bool include_send_counts)
{
  if (include_send_counts && can_driver_) {
    sample.send_retry_count = clamp_to_u32(can_driver_->getSendRetryCount());
    sample.send_fail_count = clamp_to_u32(can_driver_->getSendFailCount());
  }
  sample.write_duration_us = elapsed_us(write_start);
  debug_logger_.push(sample);
}

bool EasyArmHardware::apply_debug_logger_enabled(bool enabled, std::string & message)
{
  if (enabled == debug_logger_.is_active()) {
    debug_logger_config_.enabled = enabled;
    message = enabled ? "Debug logger is already active" : "Debug logger is already stopped";
    return true;
  }

  if (enabled) {
    debug_logger_config_.enabled = true;
    debug_logger_config_.path = timestamped_debug_log_path();
    try {
      std::filesystem::create_directories(std::filesystem::path(debug_logger_config_.path).parent_path());
    } catch (const std::filesystem::filesystem_error & error) {
      message = std::string("Failed to create debug log directory: ") + error.what();
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      debug_logger_config_.enabled = false;
      return false;
    }

    if (!debug_logger_.start(debug_logger_config_)) {
      message = "Failed to start debug logger at " + debug_logger_config_.path;
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      debug_logger_config_.enabled = false;
      return false;
    }

    debug_sequence_ = 0;
    message = "Debug logger started";
    RCLCPP_INFO(
      logger_,
      "Debug logger started: path=%s, buffer_seconds=%.1f, sample_rate=%.1f Hz",
      debug_logger_config_.path.c_str(),
      debug_logger_config_.buffer_seconds,
      debug_logger_config_.sample_rate_hz);
    return true;
  }

  debug_logger_config_.enabled = false;
  const bool was_active = debug_logger_.is_active();
  debug_logger_.stop();
  message = was_active ? "Debug logger stopped" : "Debug logger is already stopped";
  if (was_active) {
    RCLCPP_INFO(
      logger_,
      "Debug logger stopped: written=%lu, dropped=%lu, path=%s",
      static_cast<unsigned long>(debug_logger_.written_count()),
      static_cast<unsigned long>(debug_logger_.dropped_count()),
      debug_logger_config_.path.c_str());
  }
  return true;
}

DebugLoggerStatus EasyArmHardware::debug_logger_status() const
{
  std::lock_guard<std::mutex> lock(debug_state_mutex_);
  DebugLoggerStatus status;
  status.active = debug_logger_.is_active();
  status.path = debug_logger_config_.path;
  status.written_count = debug_logger_.written_count();
  status.dropped_count = debug_logger_.dropped_count();
  return status;
}

void EasyArmHardware::fill_debug_logger_status(
  easyarm_interfaces::srv::SetDebugLogger::Response & response,
  bool success,
  const std::string & message) const
{
  const auto status = debug_logger_status();
  response.success = success;
  response.message = message;
  response.active = status.active;
  response.path = status.path;
  response.written_count = status.written_count;
  response.dropped_count = status.dropped_count;
}

void EasyArmHardware::fill_debug_logger_status(
  easyarm_interfaces::srv::GetDebugLoggerStatus::Response & response,
  bool success,
  const std::string & message) const
{
  const auto status = debug_logger_status();
  response.success = success;
  response.message = message;
  response.active = status.active;
  response.path = status.path;
  response.written_count = status.written_count;
  response.dropped_count = status.dropped_count;
}

void EasyArmHardware::apply_requested_debug_logger()
{
  uint64_t generation = 0;
  bool enabled = false;
  {
    std::lock_guard<std::mutex> lock(debug_request_mutex_);
    if (!debug_request_pending_) {
      return;
    }
    generation = debug_request_generation_;
    enabled = debug_request_enabled_;
    debug_request_pending_ = false;
  }

  std::string message;
  bool success = false;
  {
    std::lock_guard<std::mutex> lock(debug_state_mutex_);
    success = apply_debug_logger_enabled(enabled, message);
  }

  {
    std::lock_guard<std::mutex> lock(debug_request_mutex_);
    debug_request_success_ = success;
    debug_request_message_ = message;
    debug_applied_generation_ = generation;
  }
  debug_request_cv_.notify_all();
}

}  // namespace easyarm_hardware
