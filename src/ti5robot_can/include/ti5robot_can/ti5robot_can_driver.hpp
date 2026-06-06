/**
 * @file ti5robot_can_driver.hpp
 * @brief 钛虎关节模组 CAN 通信驱动
 * 
 * 适配钛虎系列关节模组
 */

#ifndef TI5ROBOT_CAN__TI5ROBOT_CAN_DRIVER_HPP_
#define TI5ROBOT_CAN__TI5ROBOT_CAN_DRIVER_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace ti5robot_can
{

/**
 * @brief 电机反馈数据
 */
struct MotorFeedback
{
  uint8_t motor_id;
  double position;      // rad
  double velocity;      // rad/s
  double torque;        // Nm
  double temperature;   // °C
  uint8_t fault_code;
  bool is_valid;
  std::chrono::steady_clock::time_point last_update;
};

/**
 * @brief 钛虎 CAN 驱动类
 */
class Ti5robotCanDriver
{
public:
  /**
   * @brief 构造钛虎 CAN 驱动
   * @param can_interface CAN 接口名（例如 "can0"）
   * @param host_can_id 主机 CAN ID（默认 0x00）
   */
  explicit Ti5robotCanDriver(const std::string& can_interface, uint8_t host_can_id = 0x00);
  
  ~Ti5robotCanDriver();
  
  /**
   * @brief 初始化 CAN 接口
   * @return 成功返回 true
   */
  bool init();
  
  /**
   * @brief 关闭 CAN 接口
   */
  void close();
  
  /**
   * @brief 检查驱动是否已连接
   */
  bool isConnected() const { return socket_fd_ >= 0; }

  /**
   * @brief 使能电机
   * @param motor_id 电机 CAN ID
   * @return 成功返回 true
   */
  bool enableMotor(uint8_t motor_id);
  
  /**
   * @brief 失能电机
   * @param motor_id 电机 CAN ID
   * @return 成功返回 true
   */
  bool disableMotor(uint8_t motor_id);
  
  /**
   * @brief 获取电机反馈
   * @param motor_id 电机 CAN ID
   * @return 电机反馈数据
   */
  MotorFeedback getMotorFeedback(uint8_t motor_id);
  
  /**
   * @brief 启动接收线程
   */
  void startReceiveThread();
  
  /**
   * @brief 停止接收线程
   */
  void stopReceiveThread();

private:
  /**
   * @brief 发送 CAN 帧
   */
  bool sendFrame(const can_frame& frame);
  
  /**
   * @brief 接收 CAN 帧
   */
  bool receiveFrame(can_frame& frame, int timeout_ms = 100);
  
  /**
   * @brief 接收线程函数
   */
  void receiveThreadFunc();
  
  /**
   * @brief 从 CAN 帧解析电机反馈
   */
  void parseFeedback(const can_frame& frame);
  
  std::string can_interface_;
  uint8_t host_can_id_;
  int socket_fd_;
  
  std::mutex send_mutex_;
  std::mutex feedback_mutex_;
  
  std::atomic<bool> receive_running_;
  std::thread receive_thread_;
  
  // 电机反馈缓存（按 motor_id 索引）
  std::vector<MotorFeedback> motor_feedbacks_;
};

}  // namespace ti5robot_can

#endif  // TI5ROBOT_CAN__TI5ROBOT_CAN_DRIVER_HPP_