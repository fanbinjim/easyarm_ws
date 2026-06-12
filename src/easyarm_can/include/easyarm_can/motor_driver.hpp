/**
 * @file motor_driver.hpp
 * @brief EasyArm CAN 厂商电机驱动抽象接口。
 */

#ifndef EASYARM_CAN__MOTOR_DRIVER_HPP_
#define EASYARM_CAN__MOTOR_DRIVER_HPP_

#include <cstring>
#include <memory>
#include <string>

#include <linux/can.h>

#include "easyarm_can/socket_can_transport.hpp"
#include "easyarm_can/types.hpp"

namespace easyarm_can
{

/**
 * @brief 厂商电机驱动统一接口。
 */
class MotorDriver
{
public:
  virtual ~MotorDriver() = default;

  virtual Vendor vendor() const = 0;
  virtual ProtocolCapabilities capabilities() const = 0;
  virtual bool configure(uint8_t motor_id, const MotorModel & model) = 0;
  virtual bool clearFault(uint8_t motor_id) = 0;
  virtual bool enterHybridMode(uint8_t motor_id) = 0;
  virtual bool enableMotor(uint8_t motor_id) = 0;
  virtual bool disableMotor(uint8_t motor_id) = 0;
  virtual bool sendHybridControl(uint8_t motor_id, const HybridCommand & command) = 0;
  virtual bool sendPositionControl(uint8_t motor_id, const PositionCommand & command) = 0;
  virtual bool sendVelocityControl(uint8_t motor_id, const VelocityCommand & command) = 0;
  virtual bool parseFeedback(const canfd_frame & frame, MotorFeedback & feedback) = 0;
  virtual std::string lastError() const = 0;
};

/**
 * @brief 厂商电机驱动基类，提供通用 CAN/CAN FD 发送和错误缓存。
 */
class MotorDriverBase : public MotorDriver
{
public:
  MotorDriverBase(SocketCanTransport & transport, uint8_t host_can_id, bool is_canfd)
  : transport_(transport), host_can_id_(host_can_id), is_canfd_(is_canfd)
  {
  }

  std::string lastError() const override { return last_error_; }

protected:
  void setError(const std::string & message) { last_error_ = message; }

  bool sendCan(uint32_t can_id, const uint8_t * data, uint8_t dlc)
  {
    can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = dlc;
    if (data && dlc > 0) {
      std::memcpy(frame.data, data, dlc);
    }
    if (!transport_.send(frame)) {
      setError(transport_.lastError());
      return false;
    }
    return true;
  }

  bool sendCanFd(uint32_t can_id, const uint8_t * data, uint8_t len)
  {
    canfd_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.len = len;
    if (data && len > 0) {
      std::memcpy(frame.data, data, len);
    }
    if (!transport_.send(frame)) {
      setError(transport_.lastError());
      return false;
    }
    return true;
  }

  SocketCanTransport & transport_;
  uint8_t host_can_id_;
  bool is_canfd_{false};
  std::string last_error_;
};

std::unique_ptr<MotorDriver> createMotorDriver(
  Vendor vendor,
  SocketCanTransport & transport,
  uint8_t host_can_id,
  bool is_canfd);

}  // namespace easyarm_can

#endif  // EASYARM_CAN__MOTOR_DRIVER_HPP_
