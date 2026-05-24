#include "robstride_can/robstride_can_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

volatile std::sig_atomic_t g_running = 1;

void handleSignal(int)
{
  g_running = 0;
}

void printUsage(const char * program_name)
{
  std::cerr << "Usage: " << program_name << " "
            << "<can_interface> <motor_id> [RS00|EL05|RS05]\n"
            << "Example: ros2 run robstride_can single_motor_demo can0 0x06\n";
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

robstride_can::MotorType defaultMotorType(uint8_t motor_id)
{
  return motor_id <= 3 ? robstride_can::MotorType::RS00
                       : robstride_can::MotorType::EL05;
}

robstride_can::MotorType parseMotorType(const std::string & value)
{
  if (value == "RS00" || value == "rs00") {
    return robstride_can::MotorType::RS00;
  }
  if (value == "EL05" || value == "el05") {
    return robstride_can::MotorType::EL05;
  }
  if (value == "RS05" || value == "rs05") {
    return robstride_can::MotorType::RS05;
  }
  throw std::invalid_argument("motor_type must be RS00, EL05, or RS05");
}

bool waitFor(std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (g_running && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return g_running;
}

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
  uint8_t motor_id)
{
  const auto feedback = driver.getMotorFeedback(motor_id);
  if (!feedback.is_valid) {
    std::cout << "feedback: invalid\n";
    return;
  }

  const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - feedback.last_update).count();

  std::cout << std::fixed << std::setprecision(3)
            << "feedback: pos=" << feedback.position
            << " rad, vel=" << feedback.velocity
            << " rad/s, torque=" << feedback.torque
            << " Nm, temp=" << feedback.temperature
            << " C, mode=" << static_cast<int>(feedback.mode_state)
            << ", fault=0x" << std::hex << static_cast<int>(feedback.fault_code)
            << std::dec << ", age=" << age_ms << " ms\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  using robstride_can::MotorFeedback;
  using robstride_can::RobstrideCanDriver;
  using robstride_can::RunMode;
  using robstride_can::getMotorParams;
  using robstride_can::motorTypeName;

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  if (argc != 3 && argc != 4) {
    printUsage(argv[0]);
    return 2;
  }

  const std::string can_interface = argv[1];
  uint8_t motor_id = 0;
  robstride_can::MotorType motor_type;

  try {
    motor_id = parseMotorId(argv[2]);
    motor_type = argc == 4 ? parseMotorType(argv[3]) : defaultMotorType(motor_id);
  } catch (const std::exception & error) {
    std::cerr << "Argument error: " << error.what() << "\n";
    printUsage(argv[0]);
    return 2;
  }

  std::cout << "single_motor_demo starting: can=" << can_interface
            << ", motor_id=0x" << std::hex << static_cast<int>(motor_id)
            << std::dec << ", motor_type=" << motorTypeName(motor_type) << "\n";
  std::cout << "Press Ctrl-C to stop. The demo will disable the motor on exit.\n";

  RobstrideCanDriver driver(can_interface);
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
  waitFor(std::chrono::milliseconds(200));

  std::cout << "Setting motion-control mode...\n";
  if (!driver.setRunMode(motor_id, RunMode::MOTION_CONTROL)) {
    std::cerr << "Failed to set motion-control mode.\n";
    cleanup();
    return 1;
  }
  waitFor(std::chrono::milliseconds(100));

  std::cout << "Enabling motor...\n";
  if (!driver.enableMotor(motor_id)) {
    std::cerr << "Failed to enable motor.\n";
    cleanup();
    return 1;
  }
  motor_enabled = true;
  waitFor(std::chrono::milliseconds(300));

  MotorFeedback feedback;
  if (!waitForFeedback(driver, motor_id, feedback, std::chrono::milliseconds(1500))) {
    std::cerr << "No valid motor feedback received. Stop test to avoid blind motion.\n";
    cleanup();
    return 1;
  }

  const auto params = getMotorParams(motor_type);
  const double center_position = std::clamp(feedback.position, params.p_min, params.p_max);
  constexpr double amplitude = 0.15;
  constexpr double frequency_hz = 0.2;
  constexpr double kp = 20.0;
  constexpr double kd = 1.0;
  constexpr double torque = 0.0;
  constexpr double pi = 3.14159265358979323846;
  constexpr auto control_period = std::chrono::milliseconds(10);
  constexpr auto hold_duration = std::chrono::seconds(1);
  constexpr auto test_duration = std::chrono::seconds(12);

  std::cout << std::fixed << std::setprecision(3)
            << "Initial feedback position=" << feedback.position
            << " rad. Test center=" << center_position
            << " rad, amplitude=" << amplitude
            << " rad, duration=" << test_duration.count() << " s\n";

  auto send_position = [&](double target_position, double target_velocity) {
    target_position = std::clamp(target_position, params.p_min, params.p_max);
    return driver.sendMotionControl(
      motor_id,
      motor_type,
      target_position,
      target_velocity,
      kp,
      kd,
      torque);
  };

  std::cout << "Holding current position...\n";
  auto phase_start = std::chrono::steady_clock::now();
  while (g_running && std::chrono::steady_clock::now() - phase_start < hold_duration) {
    if (!send_position(center_position, 0.0)) {
      std::cerr << "Failed to send hold command.\n";
      cleanup();
      return 1;
    }
    std::this_thread::sleep_for(control_period);
  }

  std::cout << "Running sine position test...\n";
  const auto test_start = std::chrono::steady_clock::now();
  auto next_tick = test_start;
  int print_counter = 0;

  while (g_running && std::chrono::steady_clock::now() - test_start < test_duration) {
    const auto now = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(now - test_start).count();
    const double phase = 2.0 * pi * frequency_hz * t;
    const double target_position = center_position + amplitude * std::sin(phase);
    const double target_velocity = amplitude * 2.0 * pi * frequency_hz * std::cos(phase);

    if (!send_position(target_position, target_velocity)) {
      std::cerr << "Failed to send motion-control command.\n";
      cleanup();
      return 1;
    }

    if (++print_counter >= 50) {
      print_counter = 0;
      printFeedback(driver, motor_id);
    }

    next_tick += control_period;
    std::this_thread::sleep_until(next_tick);
  }

  std::cout << "Returning to center and holding...\n";
  phase_start = std::chrono::steady_clock::now();
  while (g_running && std::chrono::steady_clock::now() - phase_start < hold_duration) {
    if (!send_position(center_position, 0.0)) {
      std::cerr << "Failed to send final hold command.\n";
      cleanup();
      return 1;
    }
    std::this_thread::sleep_for(control_period);
  }

  cleanup();
  std::cout << "single_motor_demo finished.\n";
  return 0;
}
