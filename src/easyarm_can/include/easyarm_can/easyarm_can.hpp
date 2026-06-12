/**
 * @file easyarm_can.hpp
 * @brief EasyArm CAN 统一电机驱动门面。
 */

#ifndef EASYARM_CAN__EASYARM_CAN_HPP_
#define EASYARM_CAN__EASYARM_CAN_HPP_

#include <memory>
#include <string>
#include <vector>

#include "easyarm_can/types.hpp"

namespace easyarm_can
{

/**
 * @brief EasyArm CAN 统一入口。
 *
 * 该类不依赖 ROS 运行时；调用者负责 SocketCAN/CAN FD 接口配置和硬件安全确认。
 */
class EasyArmCan
{
public:
  /**
   * @brief 构造统一 CAN 驱动。
   * @param can_interface CAN 接口名，例如 "can0"。
   * @param host_can_id 主机 CAN ID。
   * @param is_canfd 是否启用 CAN FD 协议和 SocketCAN FD 支持。
   */
  explicit EasyArmCan(
    const std::string & can_interface,
    uint8_t host_can_id = 0x00,
    bool is_canfd = false);
  ~EasyArmCan();

  EasyArmCan(const EasyArmCan &) = delete;
  EasyArmCan & operator=(const EasyArmCan &) = delete;

  bool init();
  void close();
  bool isConnected() const;
  void setVerbose(bool verbose);

  bool configureMotor(const MotorConfig & config);
  bool configureMotors(const std::vector<MotorConfig> & configs);

  bool clearFault(uint8_t motor_id);
  bool enterHybridMode(uint8_t motor_id);
  bool enableMotor(uint8_t motor_id);
  bool disableMotor(uint8_t motor_id);
  bool sendHybridControl(uint8_t motor_id, const HybridCommand & command);
  bool sendPositionControl(uint8_t motor_id, const PositionCommand & command);
  bool sendVelocityControl(uint8_t motor_id, const VelocityCommand & command);
  MotorFeedback getMotorFeedback(uint8_t motor_id) const;

  void startReceiveThread();
  void stopReceiveThread();

  ProtocolCapabilities capabilities(uint8_t motor_id) const;
  std::string lastError() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace easyarm_can

#endif  // EASYARM_CAN__EASYARM_CAN_HPP_
