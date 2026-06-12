#include "robstride_can/robstride_can_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

volatile std::sig_atomic_t g_running = 1;
constexpr const char * kCsvPath = "/tmp/single_motor_csp_demo.csv";

void handleSignal(int)
{
  g_running = 0;
}

void printUsage(const char * program_name)
{
  std::cerr << "Usage: " << program_name << " "
            << "[can_interface] [motor_id] [cycles]\n"
            << "  can_interface default: can0\n"
            << "  motor_id default: 0x06\n"
            << "  cycles=0 means loop until Ctrl-C. Default: 0\n"
            << "Example: ros2 run robstride_can single_motor_csp_demo\n"
            << "Example: ros2 run robstride_can single_motor_csp_demo can0 0x06 2\n";
}

uint8_t parseMotorId(const std::string & value)
{
  size_t parsed_chars = 0;
  const unsigned long parsed = std::stoul(value, &parsed_chars, 0);
  if (parsed_chars != value.size() || parsed == 0 || parsed >= 16) {
    throw std::invalid_argument("motor_id must be in range 1..15");
  }
  return static_cast<uint8_t>(parsed);
}

int parseCycles(const std::string & value)
{
  size_t parsed_chars = 0;
  const int parsed = std::stoi(value, &parsed_chars, 0);
  if (parsed_chars != value.size() || parsed < 0) {
    throw std::invalid_argument("cycles must be a non-negative integer");
  }
  return parsed;
}

robstride_can::MotorType defaultMotorType(uint8_t motor_id)
{
  return motor_id <= 3 ? robstride_can::MotorType::RS00
                       : robstride_can::MotorType::EL05;
}

class CsvLogger
{
public:
  explicit CsvLogger(const std::string & path)
  : file_(path, std::ios::out | std::ios::trunc)
  {
    if (file_) {
      file_ << "timestamp,motor_id,loc_cmd,loc_state,speed_cmd,speed_state\n";
      file_ << std::fixed << std::setprecision(9);
    }
  }

  bool isOpen() const
  {
    return file_.is_open();
  }

  void log(
    robstride_can::RobstrideCanDriver & driver,
    uint8_t motor_id,
    double loc_cmd,
    double speed_cmd)
  {
    if (!file_) {
      return;
    }

    const auto now = std::chrono::system_clock::now();
    const double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
    const auto feedback = driver.getMotorFeedback(motor_id);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double loc_state = feedback.is_valid ? feedback.position : nan;
    const double speed_state = feedback.is_valid ? feedback.velocity : nan;

    file_ << timestamp << ','
          << static_cast<int>(motor_id) << ','
          << loc_cmd << ','
          << loc_state << ','
          << speed_cmd << ','
          << speed_state << '\n';
  }

private:
  std::ofstream file_;
};

bool waitForFeedback(
  robstride_can::RobstrideCanDriver & driver,
  uint8_t motor_id,
  robstride_can::MotorFeedback & feedback,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (g_running && std::chrono::steady_clock::now() < deadline) {
    feedback = driver.getMotorFeedback(motor_id);
    if (feedback.is_valid) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

void printFeedback(
  robstride_can::RobstrideCanDriver & driver,
  uint8_t motor_id,
  double target_position,
  double target_velocity)
{
  const auto feedback = driver.getMotorFeedback(motor_id);
  if (!feedback.is_valid) {
    std::cout << std::fixed << std::setprecision(3)
              << "target: pos=" << target_position
              << " rad, vel=" << target_velocity
              << " rad/s, feedback: invalid\n";
    return;
  }

  const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - feedback.last_update).count();

  std::cout << std::fixed << std::setprecision(3)
            << "target: pos=" << target_position
            << " rad, vel=" << target_velocity
            << " rad/s | feedback: pos=" << feedback.position
            << " rad, vel=" << feedback.velocity
            << " rad/s, torque=" << feedback.torque
            << " Nm, temp=" << feedback.temperature
            << " C, mode=" << static_cast<int>(feedback.mode_state)
            << ", fault=0x" << std::hex << static_cast<int>(feedback.fault_code)
            << std::dec << ", age=" << age_ms << " ms\n";
}

bool sendHold(
  robstride_can::RobstrideCanDriver & driver,
  CsvLogger & csv_logger,
  uint8_t motor_id,
  double position,
  std::chrono::milliseconds duration)
{
  constexpr auto command_period = std::chrono::milliseconds(5);
  const auto deadline = std::chrono::steady_clock::now() + duration;
  auto next_tick = std::chrono::steady_clock::now();

  while (g_running && std::chrono::steady_clock::now() < deadline) {
    constexpr double hold_speed = 0.0;
    if (!driver.sendPositionControl(motor_id, position, hold_speed)) {
      return false;
    }
    csv_logger.log(driver, motor_id, position, hold_speed);
    next_tick += command_period;
    std::this_thread::sleep_until(next_tick);
  }

  return true;
}

bool moveLinear(
  robstride_can::RobstrideCanDriver & driver,
  CsvLogger & csv_logger,
  uint8_t motor_id,
  double start_position,
  double end_position,
  double speed)
{
  constexpr auto command_period = std::chrono::milliseconds(5);
  constexpr auto feedback_period = std::chrono::seconds(1);
  constexpr double min_speed = 1.0e-6;

  const double distance = std::abs(end_position - start_position);
  if (distance < 1.0e-6) {
    return sendHold(driver, csv_logger, motor_id, end_position, std::chrono::milliseconds(300));
  }

  speed = std::max(std::abs(speed), min_speed);
  const double direction = end_position >= start_position ? 1.0 : -1.0;
  const double duration_seconds = distance / speed;
  const auto move_duration = std::chrono::duration<double>(duration_seconds);
  const auto start_time = std::chrono::steady_clock::now();
  const auto deadline = start_time + move_duration;
  auto next_tick = start_time;
  auto next_feedback = start_time + feedback_period;

  while (g_running && std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_seconds = std::chrono::duration<double>(now - start_time).count();
    const double traveled = std::min(distance, speed * elapsed_seconds);
    const double target_position = start_position + direction * traveled;
    const double target_velocity = direction * speed;

    if (!driver.sendPositionControl(motor_id, target_position, std::abs(target_velocity))) {
      return false;
    }
    csv_logger.log(driver, motor_id, target_position, target_velocity);

    if (now >= next_feedback) {
      printFeedback(driver, motor_id, target_position, target_velocity);
      next_feedback += feedback_period;
    }

    next_tick += command_period;
    std::this_thread::sleep_until(next_tick);
  }

  if (!driver.sendPositionControl(motor_id, end_position, speed)) {
    return false;
  }
  csv_logger.log(driver, motor_id, end_position, direction * speed);
  return sendHold(driver, csv_logger, motor_id, end_position, std::chrono::milliseconds(300));
}

}  // namespace

int main(int argc, char ** argv)
{
  using robstride_can::MotorFeedback;
  using robstride_can::RobstrideCanDriver;
  using robstride_can::RunMode;
  using robstride_can::motorTypeName;

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  if (argc > 4) {
    printUsage(argv[0]);
    return 2;
  }

  std::string can_interface = "can0";
  uint8_t motor_id = 0x06;
  int cycles = 0;

  try {
    if (argc >= 2) {
      can_interface = argv[1];
    }
    if (argc >= 3) {
      motor_id = parseMotorId(argv[2]);
    }
    if (argc >= 4) {
      cycles = parseCycles(argv[3]);
    }
  } catch (const std::exception & error) {
    std::cerr << "Argument error: " << error.what() << "\n";
    printUsage(argv[0]);
    return 2;
  }

  const auto motor_type = defaultMotorType(motor_id);
  constexpr double start_position = -1.570796325;
  constexpr double end_position = 1.570796325;
  constexpr double speeds[] = {1.0, 5.0};

  std::cout << "single_motor_csp_demo starting: can=" << can_interface
            << ", motor_id=0x" << std::hex << static_cast<int>(motor_id)
            << std::dec << ", motor_type=" << motorTypeName(motor_type)
            << ", cycles=" << cycles << "\n";
  std::cout << "WARNING: this demo enables one motor and commands -pi/2..pi/2 rad motion in private CSP mode.\n";
  std::cout << "Press Ctrl-C to stop. The demo will disable the motor on exit.\n";

  RobstrideCanDriver driver(can_interface);
  driver.setVerbose(false);
  CsvLogger csv_logger(kCsvPath);
  if (!csv_logger.isOpen()) {
    std::cerr << "Failed to open CSV log: " << kCsvPath << "\n";
    return 1;
  }
  std::cout << "CSV log: " << kCsvPath << "\n";
  bool motor_enabled = false;

  auto cleanup = [&]() {
    if (motor_enabled) {
      std::cout << "Disabling motor...\n";
      driver.disableMotor(motor_id, false);
      motor_enabled = false;
    }
    driver.stopReceiveThread();
    driver.close();
    std::cout << "send_retry_count=" << driver.getSendRetryCount()
              << ", send_fail_count=" << driver.getSendFailCount() << "\n";
  };

  if (!driver.init()) {
    std::cerr << "Failed to initialize CAN interface: " << can_interface << "\n";
    return 1;
  }

  driver.setMotorType(motor_id, motor_type);
  driver.startReceiveThread();

  std::cout << "Clearing faults...\n";
  if (!driver.disableMotor(motor_id, true)) {
    std::cerr << "Failed to send fault-clear disable command.\n";
    cleanup();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "Setting CSP run mode...\n";
  if (!driver.setRunMode(motor_id, RunMode::POSITION_CSP)) {
    std::cerr << "Failed to set CSP run mode.\n";
    cleanup();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::cout << "Enabling motor...\n";
  if (!driver.enableMotor(motor_id)) {
    std::cerr << "Failed to enable motor.\n";
    cleanup();
    return 1;
  }
  motor_enabled = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  MotorFeedback feedback;
  if (!waitForFeedback(driver, motor_id, feedback, std::chrono::milliseconds(1500))) {
    std::cerr << "No valid motor feedback received. Stop test to avoid blind motion.\n";
    cleanup();
    return 1;
  }

  std::cout << std::fixed << std::setprecision(3)
            << "Initial feedback position=" << feedback.position
            << " rad. Moving to -pi/2 rad before cyclic CSP test.\n";

  int completed_cycles = 0;

  std::cout << "Holding current/start position with private CSP command...\n";
  if (!sendHold(driver, csv_logger, motor_id, feedback.position, std::chrono::milliseconds(500))) {
    std::cerr << "Failed to send initial hold command.\n";
    cleanup();
    return 1;
  }

  if (!moveLinear(driver, csv_logger, motor_id, feedback.position, start_position, 1.0)) {
    std::cerr << "Failed to move to start position.\n";
    cleanup();
    return 1;
  }

  while (g_running && (cycles == 0 || completed_cycles < cycles)) {
    ++completed_cycles;
    std::cout << "Cycle " << completed_cycles
              << (cycles == 0 ? "" : ("/" + std::to_string(cycles))) << "\n";

    for (double speed : speeds) {
      std::cout << std::fixed << std::setprecision(3)
                << "CSP -pi/2 -> pi/2 rad at " << speed << " rad/s, command period=5 ms\n";
      if (!moveLinear(driver, csv_logger, motor_id, start_position, end_position, speed)) {
        std::cerr << "Failed to send CSP forward segment.\n";
        cleanup();
        return 1;
      }

      std::cout << std::fixed << std::setprecision(3)
                << "CSP pi/2 -> -pi/2 rad at " << speed << " rad/s, command period=5 ms\n";
      if (!moveLinear(driver, csv_logger, motor_id, end_position, start_position, speed)) {
        std::cerr << "Failed to send CSP return segment.\n";
        cleanup();
        return 1;
      }
    }
  }

  cleanup();
  std::cout << "single_motor_csp_demo finished.\n";
  return 0;
}
