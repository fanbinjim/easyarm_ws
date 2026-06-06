/**
 * @file xhumanoid_can_driver.hpp
 * @brief XHumanoid 电机 CAN 通信驱动骨架
 *
 * 一个驱动实例管理一个 SocketCAN 接口、一个接收线程和按 motor_id 索引的反馈缓存。
 */

#ifndef XHUMANOID_CAN__XHUMANOID_CAN_DRIVER_HPP_
#define XHUMANOID_CAN__XHUMANOID_CAN_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace xhumanoid_can
{

/**
 * @brief 电机反馈数据。
 *
 * 单位：position 为 rad，velocity 为 rad/s，torque 为 Nm，temperature 为 degC。
 */
struct MotorFeedback
{
  uint8_t motor_id{0};
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  double temperature{0.0};
  uint32_t fault_code{0};
  bool is_valid{false};
  std::chrono::steady_clock::time_point last_update{};
};

/**
 * @brief XHumanoid CAN 驱动类。
 *
 * 该类不依赖 ROS 运行时；调用者负责配置 SocketCAN 接口和保证硬件安全状态。
 */
class XhumanoidCanDriver
{
public:
  /**
   * @brief 构造 XHumanoid CAN 驱动。
   * @param can_interface CAN 接口名，例如 "can0"。
   * @param host_can_id 主机 CAN ID。
   */
  explicit XhumanoidCanDriver(const std::string & can_interface, uint8_t host_can_id = 0x00);

  ~XhumanoidCanDriver();

  /**
   * @brief 初始化 CAN socket。
   * @return 成功返回 true。
   */
  bool init();

  /**
   * @brief 关闭 CAN socket，并停止接收线程。
   */
  void close();

  /**
   * @brief 检查 CAN socket 是否已连接。
   * @return 已连接返回 true。
   */
  bool isConnected() const { return socket_fd_ >= 0; }

  /**
   * @brief 设置是否打印驱动内部信息日志。
   * @param verbose 为 true 时打印日志。
   */
  void setVerbose(bool verbose) { verbose_ = verbose; }

  /**
   * @brief 使能电机。
   * @param motor_id 电机 CAN ID。
   * @return 成功返回 true。
   */
  bool enableMotor(uint8_t motor_id);

  /**
   * @brief 失能电机。
   * @param motor_id 电机 CAN ID。
   * @return 成功返回 true。
   */
  bool disableMotor(uint8_t motor_id);

  /**
   * @brief 获取指定电机的最近一次反馈。
   * @param motor_id 电机 CAN ID。
   * @return 电机反馈数据；没有有效反馈时 is_valid 为 false。
   */
  MotorFeedback getMotorFeedback(uint8_t motor_id);

  /**
   * @brief 启动接收线程。
   */
  void startReceiveThread();

  /**
   * @brief 停止接收线程。
   */
  void stopReceiveThread();

private:
  /**
   * @brief 发送 CAN 帧。
   * @param frame CAN 帧。
   * @return 成功返回 true。
   */
  bool sendFrame(const can_frame & frame);

  /**
   * @brief 接收 CAN 帧。
   * @param frame 输出 CAN 帧。
   * @param timeout_ms 超时时间，单位 ms。
   * @return 成功收到完整帧返回 true。
   */
  bool receiveFrame(can_frame & frame, int timeout_ms = 100);

  /**
   * @brief 接收线程函数。
   */
  void receiveThreadFunc();

  /**
   * @brief 从 CAN 帧解析电机反馈。
   * @param frame CAN 帧。
   */
  void parseFeedback(const can_frame & frame);

  std::string can_interface_;
  uint8_t host_can_id_;
  int socket_fd_;
  bool verbose_{true};

  std::mutex send_mutex_;
  std::mutex feedback_mutex_;

  std::atomic<bool> receive_running_;
  std::thread receive_thread_;

  std::vector<MotorFeedback> motor_feedbacks_;
};

}  // namespace xhumanoid_can

#endif  // XHUMANOID_CAN__XHUMANOID_CAN_DRIVER_HPP_
