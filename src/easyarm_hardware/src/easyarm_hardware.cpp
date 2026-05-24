#include "easyarm_hardware/easyarm_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace easyarm_hardware
{

using robstride_can::getMotorParams;
using robstride_can::motorTypeName;

EasyArmHardware::EasyArmHardware() = default;

EasyArmHardware::~EasyArmHardware()
{
  on_shutdown(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn EasyArmHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & params = info_.hardware_parameters;
  if (params.count("can_interface")) {
    can_interface_ = params.at("can_interface");
  }
  if (params.count("host_can_id")) {
    host_can_id_ = parse_u8_parameter(params.at("host_can_id"), host_can_id_);
  }
  if (params.count("position_kp")) {
    position_kp_ = parse_double_parameter(params.at("position_kp"), position_kp_);
  }
  if (params.count("position_kd")) {
    position_kd_ = parse_double_parameter(params.at("position_kd"), position_kd_);
  }
  if (params.count("velocity_limit")) {
    velocity_limit_ = parse_double_parameter(params.at("velocity_limit"), velocity_limit_);
  }
  if (params.count("use_mock_hardware")) {
    use_mock_hardware_ = params.at("use_mock_hardware") == "true" || params.at("use_mock_hardware") == "1";
  }
  if (params.count("control_mode")) {
    desired_motor_mode_ = parse_control_mode(params.at("control_mode"));
  }
  if (params.count("smoothing_alpha")) {
    smoothing_alpha_ = std::clamp(parse_double_parameter(params.at("smoothing_alpha"), smoothing_alpha_), 0.01, 1.0);
  }
  if (params.count("max_velocity")) {
    max_velocity_ = parse_double_parameter(params.at("max_velocity"), max_velocity_);
  }
  if (params.count("max_acceleration")) {
    max_acceleration_ = parse_double_parameter(params.at("max_acceleration"), max_acceleration_);
  }
  if (params.count("control_period")) {
    control_period_ = parse_double_parameter(params.at("control_period"), control_period_);
  }

  if (!parse_joint_config(info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t num_joints = joint_configs_.size();
  hw_positions_.assign(num_joints, 0.0);
  hw_velocities_.assign(num_joints, 0.0);
  hw_efforts_.assign(num_joints, 0.0);
  hw_temperatures_.assign(num_joints, 0.0);
  hw_commands_positions_.assign(num_joints, 0.0);
  hw_commands_velocities_.assign(num_joints, 0.0);
  hw_commands_efforts_.assign(num_joints, 0.0);

  smoothed_positions_.assign(num_joints, 0.0);
  smoothed_velocities_.assign(num_joints, 0.0);
  smoothed_accelerations_.assign(num_joints, 0.0);
  last_cmd_positions_.assign(num_joints, 0.0);
  filtered_cmd_velocities_.assign(num_joints, 0.0);
  velocity_ff_stage2_.assign(num_joints, 0.0);
  vel_ma_buffer_.assign(num_joints, {0.0, 0.0, 0.0, 0.0});
  vel_ma_idx_.assign(num_joints, 0);
  velocity_settle_counter_.assign(num_joints, 0);

  for (size_t i = 0; i < num_joints; ++i) {
    for (const auto & state_interface : info_.joints[i].state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION && !state_interface.initial_value.empty()) {
        hw_positions_[i] = parse_double_parameter(state_interface.initial_value, 0.0);
        break;
      }
    }
    hw_commands_positions_[i] = hw_positions_[i];
    smoothed_positions_[i] = hw_positions_[i];
    last_cmd_positions_[i] = hw_positions_[i];
  }

  RCLCPP_INFO(
    logger_,
    "EasyArm hardware initialized: joints=%zu, can=%s, kp=%.1f, kd=%.1f, v_limit=%.1f, mock=%s",
    num_joints,
    can_interface_.c_str(),
    position_kp_,
    position_kd_,
    velocity_limit_,
    use_mock_hardware_ ? "true" : "false");
  RCLCPP_INFO(logger_, "Desired motor control mode: %s", control_mode_name(desired_motor_mode_));

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool EasyArmHardware::parse_joint_config(const hardware_interface::HardwareInfo & info)
{
  joint_configs_.clear();

  for (const auto & joint : info.joints) {
    JointConfig config;
    config.name = joint.name;

    if (joint.parameters.count("motor_id")) {
      config.motor_id = parse_u8_parameter(joint.parameters.at("motor_id"), 0);
    } else {
      RCLCPP_ERROR(logger_, "Joint %s missing required motor_id parameter", joint.name.c_str());
      return false;
    }

    if (joint.parameters.count("motor_type")) {
      config.motor_type = parse_motor_type(joint.parameters.at("motor_type"));
    } else {
      config.motor_type = config.motor_id <= 3 ? MotorType::RS00 : MotorType::EL05;
    }

    if (joint.parameters.count("position_offset")) {
      config.position_offset = parse_double_parameter(joint.parameters.at("position_offset"), 0.0);
    }
    if (joint.parameters.count("direction")) {
      config.direction = parse_double_parameter(joint.parameters.at("direction"), 1.0);
      config.direction = config.direction >= 0.0 ? 1.0 : -1.0;
    }
    for (const auto & command_interface : joint.command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_POSITION &&
          !command_interface.min.empty() && !command_interface.max.empty() &&
          command_interface.min != command_interface.max) {
        config.lower_limit = parse_double_parameter(command_interface.min, config.lower_limit);
        config.upper_limit = parse_double_parameter(command_interface.max, config.upper_limit);
        break;
      }
    }

    if (joint.parameters.count("kp")) {
      config.kp = parse_double_parameter(joint.parameters.at("kp"), 0.0);
    }
    if (joint.parameters.count("kd")) {
      config.kd = parse_double_parameter(joint.parameters.at("kd"), 0.0);
    }

    if (!has_interface(joint.command_interfaces, hardware_interface::HW_IF_POSITION)) {
      RCLCPP_ERROR(logger_, "Joint %s must expose position command interface", joint.name.c_str());
      return false;
    }
    if (!has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION)) {
      RCLCPP_ERROR(logger_, "Joint %s must expose position state interface", joint.name.c_str());
      return false;
    }

    joint_configs_.push_back(config);

    RCLCPP_INFO(
      logger_,
      "Joint %s: motor_id=%u, type=%s, direction=%.0f, offset=%.4f, limits=[%.3f, %.3f], kp=%.1f, kd=%.1f",
      config.name.c_str(),
      config.motor_id,
      motorTypeName(config.motor_type),
      config.direction,
      config.position_offset,
      config.lower_limit,
      config.upper_limit,
      config.kp,
      config.kd);
  }

  return !joint_configs_.empty();
}

hardware_interface::CallbackReturn EasyArmHardware::on_configure(const rclcpp_lifecycle::State &)
{
  if (use_mock_hardware_) {
    RCLCPP_INFO(logger_, "Mock hardware configured, skip CAN init");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  can_driver_ = std::make_unique<RobstrideCanDriver>(can_interface_, host_can_id_);
  if (!can_driver_->init()) {
    RCLCPP_ERROR(logger_, "CAN driver init failed on %s", can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & config : joint_configs_) {
    can_driver_->setMotorType(config.motor_id, config.motor_type);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  can_driver_->startReceiveThread();

  RCLCPP_INFO(logger_, "EasyArm hardware configured on %s", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EasyArmHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_motor_mode_ = MotorControlMode::MotionControl;
  if (can_driver_) {
    can_driver_->stopReceiveThread();
    can_driver_->close();
    can_driver_.reset();
  }
  RCLCPP_INFO(logger_, "EasyArm hardware cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EasyArmHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (use_mock_hardware_) {
    sync_states_to_commands();
    RCLCPP_INFO(logger_, "Mock hardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (!can_driver_ || !can_driver_->isConnected()) {
    RCLCPP_ERROR(logger_, "Cannot activate: CAN driver is not connected");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Clearing motor faults before activation");
  for (const auto & config : joint_configs_) {
    can_driver_->disableMotor(config.motor_id, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  constexpr double damping_kp = 0.0;
  constexpr double damping_kd = 2.0;

  RCLCPP_INFO(
    logger_,
    "Enabling motors in motion-control damping mode");
  for (const auto & config : joint_configs_) {
    can_driver_->disableMotor(config.motor_id, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    if (!can_driver_->setRunMode(config.motor_id, RunMode::MOTION_CONTROL)) {
      RCLCPP_ERROR(logger_, "Motor %u set motion-control mode failed", config.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    if (!can_driver_->enableMotor(config.motor_id)) {
      RCLCPP_ERROR(logger_, "Motor %u enable failed", config.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }

    can_driver_->sendMotionControl(config.motor_id, config.motor_type, 0.0, 0.0, damping_kp, damping_kd, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0));
  sync_states_to_commands();

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto & config = joint_configs_[i];
    const double motor_position = hw_positions_[i] * config.direction + config.position_offset;

    can_driver_->sendMotionControl(
      config.motor_id,
      config.motor_type,
      motor_position,
      0.0,
      damping_kp,
      damping_kd,
      0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  active_motor_mode_ = MotorControlMode::MotionControl;
  RCLCPP_INFO(logger_, "EasyArm hardware activated with %zu motors in motion-control mode", joint_configs_.size());

  if (desired_motor_mode_ == MotorControlMode::PositionCsp) {
    RCLCPP_INFO(logger_, "Switching to position_csp mode at activation");
    if (!switch_motor_mode(MotorControlMode::PositionCsp)) {
      RCLCPP_ERROR(logger_, "Failed to switch to position_csp mode at activation");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "EasyArm hardware now in position_csp mode");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EasyArmHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (!use_mock_hardware_ && can_driver_) {
    send_damping_before_disable();
    for (const auto & config : joint_configs_) {
      can_driver_->disableMotor(config.motor_id, false);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  active_motor_mode_ = MotorControlMode::MotionControl;
  RCLCPP_INFO(logger_, "EasyArm hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EasyArmHardware::on_shutdown(const rclcpp_lifecycle::State &)
{
  on_deactivate(rclcpp_lifecycle::State());
  on_cleanup(rclcpp_lifecycle::State());
  RCLCPP_INFO(logger_, "EasyArm hardware shutdown");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EasyArmHardware::on_error(const rclcpp_lifecycle::State &)
{
  if (!use_mock_hardware_ && can_driver_) {
    send_damping_before_disable();
    for (const auto & config : joint_configs_) {
      can_driver_->disableMotor(config.motor_id, true);
    }
  }
  RCLCPP_ERROR(logger_, "EasyArm hardware error");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EasyArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    state_interfaces.emplace_back(joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(joint_configs_[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
    state_interfaces.emplace_back(joint_configs_[i].name, "temperature", &hw_temperatures_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EasyArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    command_interfaces.emplace_back(joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]);
    command_interfaces.emplace_back(joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]);
    command_interfaces.emplace_back(joint_configs_[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type EasyArmHardware::prepare_command_mode_switch(
  const std::vector<std::string> &,
  const std::vector<std::string> &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EasyArmHardware::perform_command_mode_switch(
  const std::vector<std::string> &,
  const std::vector<std::string> &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EasyArmHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (use_mock_hardware_) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      hw_positions_[i] = smoothed_positions_[i];
      hw_velocities_[i] = smoothed_velocities_[i];
      hw_efforts_[i] = 0.0;
      hw_temperatures_[i] = 25.0;
    }
    return hardware_interface::return_type::OK;
  }

  if (!can_driver_) {
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto & config = joint_configs_[i];
    const auto feedback = can_driver_->getMotorFeedback(config.motor_id);
    if (feedback.is_valid) {
      const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - feedback.last_update).count();
      if (age_ms > 50) {
        static int stale_warn = 0;
        if (stale_warn++ % 200 == 0) {
          RCLCPP_WARN(logger_, "Motor %u feedback stale: %ld ms", config.motor_id, age_ms);
        }
      }
      hw_positions_[i] = (feedback.position - config.position_offset) * config.direction;
      hw_velocities_[i] = feedback.velocity * config.direction;
      hw_efforts_[i] = feedback.torque * config.direction;
      hw_temperatures_[i] = feedback.temperature;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EasyArmHardware::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  static int write_counter = 0;
  static uint64_t frequency_counter = 0;
  static auto frequency_window_start = std::chrono::steady_clock::now();

  if (use_mock_hardware_) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      hw_commands_positions_[i] = clamp_joint_position(i, hw_commands_positions_[i]);
      smoothed_positions_[i] = hw_commands_positions_[i];
      smoothed_velocities_[i] = 0.0;
    }
    return hardware_interface::return_type::OK;
  }

  if (!can_driver_ || !can_driver_->isConnected()) {
    return hardware_interface::return_type::ERROR;
  }

  ++frequency_counter;
  const auto frequency_now = std::chrono::steady_clock::now();
  const auto frequency_elapsed = frequency_now - frequency_window_start;
  if (frequency_elapsed >= std::chrono::seconds(5)) {
    const double elapsed_seconds = std::chrono::duration<double>(frequency_elapsed).count();
    RCLCPP_INFO(
      logger_,
      "write() frequency: %.2f Hz (%lu calls in %.2f s)",
      static_cast<double>(frequency_counter) / elapsed_seconds,
      frequency_counter,
      elapsed_seconds);
    frequency_counter = 0;
    frequency_window_start = frequency_now;
  }

  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = control_period_;
  } else if (dt > control_period_ * 2.0) {
    dt = control_period_ * 2.0;
  }

  auto write_deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(4500);

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    if (std::chrono::steady_clock::now() > write_deadline) {
      static int deadline_warn = 0;
      if (deadline_warn++ % 200 == 0) {
        RCLCPP_WARN(logger_, "write() deadline reached, skipping joint %zu and later", i);
      }
      break;
    }

    const auto & config = joint_configs_[i];
    const double target_position = clamp_joint_position(i, hw_commands_positions_[i]);

    smoothed_positions_[i] = smoothing_alpha_ * target_position + (1.0 - smoothing_alpha_) * smoothed_positions_[i];

    double smoothed_velocity = (smoothed_positions_[i] - last_cmd_positions_[i]) / dt;
    last_cmd_positions_[i] = smoothed_positions_[i];

    const double max_velocity_change = max_acceleration_ * dt;
    const double acceleration_delta = smoothed_velocity - smoothed_velocities_[i];
    if (std::abs(acceleration_delta) > max_velocity_change) {
      smoothed_velocity = smoothed_velocities_[i] + max_velocity_change * (acceleration_delta > 0.0 ? 1.0 : -1.0);
    }

    smoothed_velocity = std::clamp(smoothed_velocity, -max_velocity_, max_velocity_);
    smoothed_accelerations_[i] = (smoothed_velocity - smoothed_velocities_[i]) / dt;
    smoothed_velocities_[i] = smoothed_velocity;

    vel_ma_buffer_[i][vel_ma_idx_[i]] = smoothed_velocity;
    vel_ma_idx_[i] = (vel_ma_idx_[i] + 1) % 4;

    double ma_velocity = 0.0;
    for (double value : vel_ma_buffer_[i]) {
      ma_velocity += value;
    }
    ma_velocity *= 0.25;

    double filtered_velocity = std::clamp(ma_velocity, -velocity_limit_, velocity_limit_);
    const bool stopped = std::abs(filtered_velocity) < 0.02;
    if (stopped) {
      if (++velocity_settle_counter_[i] >= 1) {
        filtered_cmd_velocities_[i] = 0.0;
        velocity_ff_stage2_[i] = 0.0;
        filtered_velocity = 0.0;
      }
    } else {
      velocity_settle_counter_[i] = 0;
    }

    constexpr double alpha = 0.18;
    filtered_cmd_velocities_[i] = alpha * filtered_velocity + (1.0 - alpha) * filtered_cmd_velocities_[i];
    velocity_ff_stage2_[i] = alpha * filtered_cmd_velocities_[i] + (1.0 - alpha) * velocity_ff_stage2_[i];

    const double motor_position = smoothed_positions_[i] * config.direction + config.position_offset;
    const double motor_velocity = velocity_ff_stage2_[i] * config.direction;
    const double motor_torque = hw_commands_efforts_[i] * config.direction;
    const bool allow_position_control = true;
    const double command_kp = allow_position_control ? 20.0 : 0.0;
    constexpr double command_kd = 5.0;
    const double command_velocity = allow_position_control ? motor_velocity : 0.0;
    const double command_torque = allow_position_control ? motor_torque : 0.0;

    auto motor_params = getMotorParams(config.motor_type);
    const double clamped_motor_position = std::clamp(motor_position, motor_params.p_min, motor_params.p_max);

    // if (write_counter % 200 == 0) {
    //   RCLCPP_INFO(
    //     logger_,
    //     "write() %s command joint=%s motor_id=%u type=%s pos=%.6f vel=%.6f kp=%.3f kd=%.3f torque=%.6f",
    //     allow_position_control ? "position" : "damping",
    //     config.name.c_str(),
    //     config.motor_id,
    //     motorTypeName(config.motor_type),
    //     clamped_motor_position,
    //     command_velocity,
    //     command_kp,
    //     command_kd,
    //     command_torque);
    // }

    const bool command_sent = active_motor_mode_ == MotorControlMode::PositionCsp ?
      can_driver_->setPositionCSP(config.motor_id, clamped_motor_position) :
      can_driver_->sendMotionControl(
        config.motor_id,
        config.motor_type,
        clamped_motor_position,
        command_velocity,
        command_kp,
        command_kd,
        command_torque);
    if (!command_sent) {
      static int warn_counter = 0;
      if (warn_counter++ % 1000 == 0) {
        RCLCPP_WARN(logger_, "Failed to send command to motor %u", config.motor_id);
      }
    }

    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }

  write_counter++;
  return hardware_interface::return_type::OK;
}

bool EasyArmHardware::has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string & name) const
{
  return std::any_of(interfaces.begin(), interfaces.end(), [&name](const auto & interface) {
    return interface.name == name;
  });
}

MotorControlMode EasyArmHardware::parse_control_mode(const std::string & value) const
{
  if (value == "position_csp" || value == "csp" || value == "POSITION_CSP") {
    return MotorControlMode::PositionCsp;
  }
  if (value == "motion_control" || value == "motion" || value == "MOTION_CONTROL") {
    return MotorControlMode::MotionControl;
  }

  RCLCPP_WARN(logger_, "Unknown control_mode '%s', fallback to motion_control", value.c_str());
  return MotorControlMode::MotionControl;
}

const char * EasyArmHardware::control_mode_name(MotorControlMode mode) const
{
  switch (mode) {
    case MotorControlMode::PositionCsp:
      return "position_csp";
    case MotorControlMode::MotionControl:
    default:
      return "motion_control";
  }
}

MotorType EasyArmHardware::parse_motor_type(const std::string & value) const
{
  if (value == "RS00" || value == "rs00") {
    return MotorType::RS00;
  }
  if (value == "EL05" || value == "el05") {
    return MotorType::EL05;
  }
  if (value == "RS05" || value == "rs05") {
    return MotorType::RS05;
  }
  RCLCPP_WARN(logger_, "Unknown motor_type '%s', fallback to RS00", value.c_str());
  return MotorType::RS00;
}

uint8_t EasyArmHardware::parse_u8_parameter(const std::string & value, uint8_t default_value) const
{
  try {
    int base = 10;
    if (value.rfind("0x", 0) == 0 || value.rfind("0X", 0) == 0) {
      base = 16;
    }
    const auto parsed = std::stoul(value, nullptr, base);
    if (parsed > std::numeric_limits<uint8_t>::max()) {
      return default_value;
    }
    return static_cast<uint8_t>(parsed);
  } catch (const std::exception &) {
    return default_value;
  }
}

double EasyArmHardware::parse_double_parameter(const std::string & value, double default_value) const
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return default_value;
  }
}

bool EasyArmHardware::switch_motor_mode(MotorControlMode mode)
{
  if (!can_driver_) {
    return false;
  }

  RCLCPP_INFO(logger_, "Switching motors to %s mode from write()", control_mode_name(mode));

  const auto run_mode = mode == MotorControlMode::PositionCsp ?
    RunMode::POSITION_CSP : RunMode::MOTION_CONTROL;

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto & config = joint_configs_[i];

    can_driver_->disableMotor(config.motor_id, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!can_driver_->setRunMode(config.motor_id, run_mode)) {
      RCLCPP_ERROR(logger_, "Motor %u set %s mode failed", config.motor_id, control_mode_name(mode));
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (mode == MotorControlMode::PositionCsp) {
      can_driver_->setVelocityLimit(config.motor_id, velocity_limit_);
      can_driver_->setPositionKp(config.motor_id, 40.0);
      can_driver_->setSpeedKp(config.motor_id, 6.0);
      can_driver_->setSpeedKi(config.motor_id, 0.02);
      can_driver_->setSpeedFilterGain(config.motor_id, 0.1);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    if (!can_driver_->enableMotor(config.motor_id)) {
      RCLCPP_ERROR(logger_, "Motor %u enable failed after mode switch", config.motor_id);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (mode == MotorControlMode::PositionCsp) {
      const double motor_position = hw_positions_[i] * config.direction + config.position_offset;
      auto motor_params = getMotorParams(config.motor_type);
      const double clamped_motor_position = std::clamp(motor_position, motor_params.p_min, motor_params.p_max);
      can_driver_->setPositionCSP(config.motor_id, clamped_motor_position);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  }

  active_motor_mode_ = mode;
  return true;
}

void EasyArmHardware::sync_states_to_commands()
{
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    hw_commands_positions_[i] = hw_positions_[i];
    hw_commands_velocities_[i] = 0.0;
    hw_commands_efforts_[i] = 0.0;
    smoothed_positions_[i] = hw_positions_[i];
    smoothed_velocities_[i] = 0.0;
    smoothed_accelerations_[i] = 0.0;
    last_cmd_positions_[i] = hw_positions_[i];
    filtered_cmd_velocities_[i] = 0.0;
    velocity_ff_stage2_[i] = 0.0;
    velocity_settle_counter_[i] = 0;
  }
}

void EasyArmHardware::send_damping_before_disable()
{
  if (!can_driver_ || joint_configs_.empty()) {
    return;
  }

  if (active_motor_mode_ == MotorControlMode::PositionCsp) {
    RCLCPP_INFO(logger_, "Motors in position_csp mode, skipping mode switch before disable");
    return;
  }

  constexpr double damping_kp = 0.0;
  constexpr double damping_kd = 8.0;
  constexpr double damping_velocity = 0.0;
  constexpr double damping_torque = 0.0;
  constexpr auto damping_duration = std::chrono::milliseconds(500);
  constexpr auto damping_period = std::chrono::milliseconds(20);

  RCLCPP_INFO(
    logger_,
    "Sending damping commands before disable: duration=%.1f s, kp=%.1f, kd=%.1f",
    std::chrono::duration<double>(damping_duration).count(),
    damping_kp,
    damping_kd);

  const auto deadline = std::chrono::steady_clock::now() + damping_duration;

  while (std::chrono::steady_clock::now() < deadline) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      const auto & config = joint_configs_[i];
      const double motor_position = hw_positions_[i] * config.direction + config.position_offset;
      auto motor_params = getMotorParams(config.motor_type);
      const double clamped_motor_position = std::clamp(motor_position, motor_params.p_min, motor_params.p_max);

      can_driver_->sendMotionControl(
        config.motor_id,
        config.motor_type,
        clamped_motor_position,
        damping_velocity,
        damping_kp,
        damping_kd,
        damping_torque);
    }
    std::this_thread::sleep_for(damping_period);
  }
}

double EasyArmHardware::clamp_joint_position(size_t joint_index, double position) const
{
  if (joint_index >= joint_configs_.size()) {
    return position;
  }
  const auto & config = joint_configs_[joint_index];
  return std::clamp(position, config.lower_limit, config.upper_limit);
}

}  // namespace easyarm_hardware

PLUGINLIB_EXPORT_CLASS(easyarm_hardware::EasyArmHardware, hardware_interface::SystemInterface)
