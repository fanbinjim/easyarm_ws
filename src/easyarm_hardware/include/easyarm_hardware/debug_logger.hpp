#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace easyarm_hardware
{

constexpr size_t kDebugJointCount = 6;
constexpr uint8_t kDebugNoSkippedJoint = 0xFF;

/**
 * @brief 调试日志配置。
 *
 * 单位：buffer_seconds 为 s，sample_rate_hz 为 Hz。
 */
struct DebugLoggerConfig
{
  bool enabled{false};
  std::string path{};
  double buffer_seconds{60.0};
  double sample_rate_hz{250.0};
};

/**
 * @brief 单关节调试数据。
 *
 * 单位：position 为 rad，velocity 为 rad/s，effort/torque 为 Nm。
 */
struct JointDebugData
{
  double state_position{0.0};
  double state_velocity{0.0};
  double state_effort{0.0};
  double command_position{0.0};
  double command_velocity{0.0};
  double command_effort{0.0};
  double smoothed_position{0.0};
  double smoothed_velocity{0.0};
  double motor_position{0.0};
  double motor_velocity{0.0};
  double motor_torque{0.0};
  double kp{0.0};
  double kd{0.0};
  uint8_t motor_id{0};
  uint8_t send_ok{0};
  uint8_t reserved[6]{};
};

/**
 * @brief 单个 hardware write 周期的调试数据。
 *
 * 单位：steady_time_ns 为 ns，period_s 为 s，write_duration_us 为 us。
 */
struct HardwareDebugSample
{
  uint64_t seq{0};
  int64_t steady_time_ns{0};
  double period_s{0.0};
  uint8_t hardware_mode{0};
  uint8_t motor_mode{0};
  uint8_t skipped_from_joint{kDebugNoSkippedJoint};
  uint8_t reserved0{0};
  uint32_t send_retry_count{0};
  uint32_t send_fail_count{0};
  uint32_t dropped_before{0};
  uint32_t write_duration_us{0};
  uint32_t reserved1{0};
  std::array<JointDebugData, kDebugJointCount> joints{};
};

struct DebugLogFileHeader
{
  char magic[8]{};
  uint32_t version{1};
  uint32_t header_size{0};
  uint32_t sample_size{0};
  uint32_t joint_count{0};
  int64_t start_steady_time_ns{0};
  int64_t start_system_time_ns{0};
  uint8_t reserved[32]{};
};

static_assert(sizeof(JointDebugData) == 112, "JointDebugData binary layout changed");
static_assert(sizeof(HardwareDebugSample) == 720, "HardwareDebugSample binary layout changed");
static_assert(sizeof(DebugLogFileHeader) == 72, "DebugLogFileHeader binary layout changed");

class DebugLogger
{
public:
  DebugLogger() = default;
  ~DebugLogger();

  DebugLogger(const DebugLogger &) = delete;
  DebugLogger & operator=(const DebugLogger &) = delete;

  bool start(const DebugLoggerConfig & config);
  void stop();
  bool push(const HardwareDebugSample & sample);

  uint64_t dropped_count() const;
  uint64_t written_count() const;
  bool is_active() const;

private:
  bool pop(HardwareDebugSample & sample);
  void run();
  bool write_header();
  static int64_t steady_time_ns();
  static int64_t system_time_ns();

  DebugLoggerConfig config_;
  std::vector<HardwareDebugSample> buffer_;
  size_t capacity_{0};
  std::atomic<size_t> head_{0};
  std::atomic<size_t> tail_{0};
  std::atomic<bool> active_{false};
  std::atomic<uint64_t> dropped_count_{0};
  std::atomic<uint64_t> written_count_{0};
  std::ofstream file_;
  std::thread thread_;
};

}  // namespace easyarm_hardware
