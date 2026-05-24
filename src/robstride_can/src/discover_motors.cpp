#include "robstride_can/robstride_can_driver.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace
{

constexpr size_t kMaxDiscoveredMotors = 256;
constexpr auto kProbeDelay = std::chrono::milliseconds(20);
constexpr auto kFeedbackCollectTime = std::chrono::milliseconds(50);
constexpr auto kProbeFeedbackTimeout = std::chrono::milliseconds(150);
constexpr auto kMonitorCollectTime = std::chrono::milliseconds(50);
constexpr auto kSoftCommandPeriod = std::chrono::milliseconds(100);
constexpr auto kPrintPeriod = std::chrono::milliseconds(500);

volatile std::sig_atomic_t g_running = 1;
std::atomic_bool g_stop_scan{false};

/**
 * @brief 处理 Ctrl-C / SIGTERM，通知主循环退出。
 */
void handleSignal(int)
{
  g_running = 0;
}

/**
 * @brief 打印命令行使用说明。
 */
void printUsage(const char * program_name)
{
  std::cerr
    << "Usage: " << program_name << " [can_interface] [motor_id0 motor_id1 ...]\n"
    << "Examples:\n"
    << "  ros2 run robstride_can discover_motors\n"
    << "  ros2 run robstride_can discover_motors can0\n"
    << "  ros2 run robstride_can discover_motors can0 0x06 7 8\n"
    << "  ros2 run robstride_can discover_motors \"\" 0x06\n";
}

/**
 * @brief 解析电机 CAN ID 参数。
 *
 * 支持十进制和 0x 前缀十六进制输入，合法范围为 0~255。
 */
uint8_t parseMotorId(const std::string & value)
{
  size_t parsed_chars = 0;
  const unsigned long parsed = std::stoul(value, &parsed_chars, 0);
  if (parsed_chars != value.size() || parsed > 255) {
    throw std::invalid_argument("motor_id must be in range 0..255");
  }
  return static_cast<uint8_t>(parsed);
}

/**
 * @brief 将有效反馈写入发现列表。
 *
 * 首次发现某个电机时打印一行发现日志；后续反馈只更新缓存。
 */
void addFeedback(
  const robstride_can::MotorFeedback & feedback,
  std::map<uint8_t, robstride_can::MotorFeedback> & discovered)
{
  if (!feedback.is_valid) {
    return;
  }

  const bool is_new_motor = discovered.count(feedback.motor_id) == 0;
  if (is_new_motor && discovered.size() >= kMaxDiscoveredMotors) {
    return;
  }

  discovered[feedback.motor_id] = feedback;
  if (is_new_motor) {
    std::cout << "发现电机: id=0x" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(feedback.motor_id) << std::dec << std::setfill(' ')
              << ", angle=" << std::fixed << std::setprecision(3) << feedback.position
              << " rad, temp=" << feedback.temperature
              << " C, mode=" << static_cast<int>(feedback.mode_state)
              << ", fault=0x" << std::hex << static_cast<int>(feedback.fault_code)
              << std::dec << "\n";
  }
}

/**
 * @brief 在给定时间内轮询一组电机 ID 的最新反馈。
 *
 * 这里读取的是 RobstrideCanDriver 接收线程维护的反馈缓存。
 */
void collectFeedback(
  robstride_can::RobstrideCanDriver & driver,
  const std::vector<uint8_t> & motor_ids,
  std::map<uint8_t, robstride_can::MotorFeedback> & discovered,
  std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (g_running && std::chrono::steady_clock::now() < deadline) {
    for (const uint8_t motor_id : motor_ids) {
      addFeedback(driver.getMotorFeedback(motor_id), discovered);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

/**
 * @brief 发送运控模式软状态指令。
 *
 * Kp=0 表示不跟踪位置，Kd=4 用于增加阻尼，torque=0 避免主动输出力矩。
 */
bool sendSoftMotionCommand(
  robstride_can::RobstrideCanDriver & driver,
  uint8_t motor_id)
{
  // 发现程序不识别电机型号。这里仅借用 EL05 的编码范围发送 Kp=0/Kd=4 的软状态指令；
  // 在当前支持的 RS00/EL05/RS05 中，位置范围和 Kp/Kd 范围一致，且力矩为 0。
  constexpr auto kProbeMotorType = robstride_can::MotorType::EL05;
  return driver.sendMotionControl(
    motor_id,
    kProbeMotorType,
    0.0,  // 位置无关紧要，因为 Kp=0
    0.0,  // velocity = 0
    0.0,  // Kp = 0，不跟踪位置
    4.0,  // Kd = 4.0，提高阻尼，防止抖动
    0.0); // torque = 0
}

/**
 * @brief 记录程序中已经使能过的电机 ID。
 *
 * 退出时只对这些电机发送失能命令，避免重复失能和遗漏。
 */
void rememberEnabledMotor(
  uint8_t motor_id,
  std::vector<uint8_t> & enabled_motor_ids,
  std::array<bool, 256> & enabled_seen)
{
  if (enabled_seen[motor_id]) {
    return;
  }
  enabled_seen[motor_id] = true;
  enabled_motor_ids.push_back(motor_id);
}

/**
 * @brief 退出前失能本程序使能过的全部电机。
 */
void disableEnabledMotors(
  robstride_can::RobstrideCanDriver & driver,
  const std::vector<uint8_t> & enabled_motor_ids)
{
  if (enabled_motor_ids.empty()) {
    return;
  }

  std::cout << "退出前失能已使能过的电机...\n";
  for (uint8_t motor_id : enabled_motor_ids) {
    driver.disableMotor(motor_id, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

/**
 * @brief 如果 stdout 是交互终端，则清屏并把光标移动到左上角。
 */
void clearTerminalIfInteractive()
{
  if (isatty(STDOUT_FILENO)) {
    std::cout << "\033[2J\033[H";
  }
}

/**
 * @brief 打印实时监控状态页。
 *
 * refresh_terminal 为 true 时会先清屏，用于持续刷新显示。
 */
void printDiscoveredMotors(
  const std::map<uint8_t, robstride_can::MotorFeedback> & discovered,
  bool refresh_terminal = false)
{
  if (refresh_terminal) {
    clearTerminalIfInteractive();
  }

  std::cout << "\n当前发现电机: " << discovered.size() << " 个\n";
  for (const auto & item : discovered) {
    const auto & feedback = item.second;
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - feedback.last_update).count();

    std::cout << "  id=0x" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(feedback.motor_id) << std::dec << std::setfill(' ')
              << ", angle=" << std::fixed << std::setprecision(3) << feedback.position
              << " rad, temp=" << feedback.temperature
              << " C, mode=" << static_cast<int>(feedback.mode_state)
              << ", fault=0x" << std::hex << static_cast<int>(feedback.fault_code)
              << std::dec << ", age=" << age_ms << " ms\n";
  }

  std::cout << "按 Ctrl-C 退出...\n";
}

/**
 * @brief 打印扫描阶段状态页。
 *
 * 显示当前正在扫描的 ID、扫描进度、已发现电机状态，以及按键退出提示。
 */
void printScanStatus(
  uint8_t motor_id,
  size_t candidate_index,
  size_t candidate_count,
  const std::map<uint8_t, robstride_can::MotorFeedback> & discovered)
{
  clearTerminalIfInteractive();
  std::cout << "\n扫描 id=0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(motor_id) << std::dec << std::setfill(' ')
            << " (" << candidate_index << "/" << candidate_count << ")\n";
  std::cout << "当前发现电机: " << discovered.size() << " 个\n";

  for (const auto & item : discovered) {
    const auto & feedback = item.second;
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - feedback.last_update).count();

    std::cout << "  id=0x" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(feedback.motor_id) << std::dec << std::setfill(' ')
              << ", angle=" << std::fixed << std::setprecision(3) << feedback.position
              << " rad, temp=" << feedback.temperature
              << " C, mode=" << static_cast<int>(feedback.mode_state)
              << ", fault=0x" << std::hex << static_cast<int>(feedback.fault_code)
              << std::dec << ", age=" << age_ms << " ms\n";
  }

  std::cout << "按 Enter 退出扫描...\n";
}

/**
 * @brief 根据命令行参数生成待扫描电机 ID 列表。
 *
 * 未指定 ID 时默认扫描 0~255；指定多个 ID 时会去重并保留输入顺序。
 */
std::vector<uint8_t> buildScanList(int argc, char ** argv)
{
  std::vector<uint8_t> motor_ids;
  if (argc <= 2) {
    motor_ids.reserve(256);
    for (int id = 0; id <= 255; ++id) {
      motor_ids.push_back(static_cast<uint8_t>(id));
    }
    return motor_ids;
  }

  motor_ids.reserve(static_cast<size_t>(argc - 2));
  std::array<bool, 256> seen{};
  for (int i = 2; i < argc; ++i) {
    const uint8_t motor_id = parseMotorId(argv[i]);
    if (!seen[motor_id]) {
      motor_ids.push_back(motor_id);
      seen[motor_id] = true;
    }
  }
  return motor_ids;
}

}  // namespace

/**
 * @brief discover_motors 程序入口。
 *
 * 扫描候选 ID，发现有反馈的电机后进入持续监控；退出时失能本程序使能过的电机。
 */
int main(int argc, char ** argv)
{
  using robstride_can::RobstrideCanDriver;
  using robstride_can::RunMode;

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  if (argc > 258) {
    printUsage(argv[0]);
    return 2;
  }

  std::string can_interface = "can0";
  if (argc >= 2 && std::string(argv[1]).size() > 0) {
    can_interface = argv[1];
  }

  std::vector<uint8_t> motor_ids;
  try {
    motor_ids = buildScanList(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "参数错误: " << error.what() << "\n";
    printUsage(argv[0]);
    return 2;
  }

  std::cout << "discover_motors starting: can=" << can_interface
            << ", candidates=" << motor_ids.size()
            << ", max_discovered=" << kMaxDiscoveredMotors << "\n";
  std::cout << "按 Ctrl-C 可停止扫描。发现电机后会发送 Kp=0、Kd=4 的运控软状态指令。\n";

  RobstrideCanDriver driver(can_interface);
  driver.setVerbose(false);
  if (!driver.init()) {
    std::cerr << "CAN 驱动初始化失败: " << can_interface << "\n";
    return 1;
  }

  driver.startReceiveThread();
  std::thread enter_thread;
  if (isatty(STDIN_FILENO)) {
    enter_thread = std::thread([]() {
      std::cin.get();
      g_stop_scan = true;
    });
  }

  std::map<uint8_t, robstride_can::MotorFeedback> discovered;
  std::vector<uint8_t> enabled_motor_ids;
  std::array<bool, 256> enabled_seen{};

  auto collect_feedback = [&]() {
    collectFeedback(driver, motor_ids, discovered, kFeedbackCollectTime);
  };

  collect_feedback();

  for (size_t i = 0; i < motor_ids.size(); ++i) {
    const uint8_t motor_id = motor_ids[i];
    if (!g_running || discovered.size() >= kMaxDiscoveredMotors) {
      break;
    }
    if (g_stop_scan.load(std::memory_order_relaxed)) {
      std::cout << "\n收到 Enter，停止继续扫描。\n";
      break;
    }

    printScanStatus(motor_id, i + 1, motor_ids.size(), discovered);

    driver.setRunMode(motor_id, RunMode::MOTION_CONTROL);
    std::this_thread::sleep_for(kProbeDelay);
    const bool enable_sent = driver.enableMotor(motor_id);
    std::this_thread::sleep_for(kProbeDelay);

    sendSoftMotionCommand(driver, motor_id);

    std::vector<uint8_t> probe_ids{motor_id};
    collectFeedback(driver, probe_ids, discovered, kProbeFeedbackTimeout);

    if (discovered.count(motor_id) > 0) {
      if (enable_sent) {
        rememberEnabledMotor(motor_id, enabled_motor_ids, enabled_seen);
      }
    } else if (enable_sent) {
      driver.disableMotor(motor_id, false);
      std::cout << "\n  id=0x" << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(motor_id) << std::dec << std::setfill(' ')
                << " 未收到反馈，已发送失能\n";
    }

    if (g_stop_scan.load(std::memory_order_relaxed)) {
      std::cout << "\n收到 Enter，停止继续扫描。\n";
      break;
    }
  }

  if (enter_thread.joinable()) {
    enter_thread.detach();
  }

  if (discovered.size() >= kMaxDiscoveredMotors) {
    std::cout << "已发现 " << kMaxDiscoveredMotors << " 个电机，停止继续扫描。\n";
  }

  printDiscoveredMotors(discovered);

  if (!discovered.empty() && g_running) {
    std::cout << "\n进入持续读取模式，按 Ctrl-C 退出。\n";
    auto next_soft_command = std::chrono::steady_clock::now();
    auto next_print = std::chrono::steady_clock::now();

    while (g_running) {
      std::vector<uint8_t> discovered_ids;
      discovered_ids.reserve(discovered.size());
      for (const auto & item : discovered) {
        discovered_ids.push_back(item.first);
      }
      collectFeedback(driver, discovered_ids, discovered, kMonitorCollectTime);

      const auto now = std::chrono::steady_clock::now();
      if (now >= next_soft_command) {
        for (const auto & item : discovered) {
          sendSoftMotionCommand(driver, item.first);
        }
        next_soft_command = now + kSoftCommandPeriod;
      }

      if (now >= next_print) {
        printDiscoveredMotors(discovered, true);
        next_print = now + kPrintPeriod;
      }
    }
  }

  disableEnabledMotors(driver, enabled_motor_ids);
  driver.stopReceiveThread();
  driver.close();
  std::cout << "send_retry_count=" << driver.getSendRetryCount()
            << ", send_fail_count=" << driver.getSendFailCount() << "\n";
  return 0;
}
