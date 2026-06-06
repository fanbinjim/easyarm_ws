/**
 * @file protocols.hpp
 * @brief EasyArm CAN 厂商协议后端。
 */

#ifndef EASYARM_CAN__PROTOCOLS_HPP_
#define EASYARM_CAN__PROTOCOLS_HPP_

#include <memory>
#include <string>

#include <linux/can.h>

#include "easyarm_can/socket_can_transport.hpp"
#include "easyarm_can/types.hpp"

namespace easyarm_can
{

class MotorProtocol
{
public:
  virtual ~MotorProtocol() = default;

  virtual Vendor vendor() const = 0;
  virtual ProtocolCapabilities capabilities() const = 0;
  virtual bool configure(uint8_t motor_id, const MotorModel & model) = 0;
  virtual bool clearFault(uint8_t motor_id) = 0;
  virtual bool enterHybridMode(uint8_t motor_id) = 0;
  virtual bool enableMotor(uint8_t motor_id) = 0;
  virtual bool disableMotor(uint8_t motor_id) = 0;
  virtual bool sendHybridControl(uint8_t motor_id, const HybridCommand & command) = 0;
  virtual bool parseFeedback(const canfd_frame & frame, MotorFeedback & feedback) = 0;
  virtual std::string lastError() const = 0;
};

std::unique_ptr<MotorProtocol> createProtocol(
  Vendor vendor,
  SocketCanTransport & transport,
  uint8_t host_can_id);

}  // namespace easyarm_can

#endif  // EASYARM_CAN__PROTOCOLS_HPP_
