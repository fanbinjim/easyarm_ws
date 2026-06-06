/**
 * @file ti5robot_driver.hpp
 * @brief 钛虎机器人电机协议驱动。
 */

#ifndef EASYARM_CAN__DRIVERS__TI5ROBOT_DRIVER_HPP_
#define EASYARM_CAN__DRIVERS__TI5ROBOT_DRIVER_HPP_

#include <map>

#include "easyarm_can/motor_driver.hpp"

namespace easyarm_can
{

class Ti5robotDriver final : public MotorDriverBase
{
public:
  using MotorDriverBase::MotorDriverBase;

  Vendor vendor() const override;
  ProtocolCapabilities capabilities() const override;
  bool configure(uint8_t motor_id, const MotorModel & model) override;
  bool clearFault(uint8_t motor_id) override;
  bool enterHybridMode(uint8_t motor_id) override;
  bool enableMotor(uint8_t motor_id) override;
  bool disableMotor(uint8_t motor_id) override;
  bool sendHybridControl(uint8_t motor_id, const HybridCommand & command) override;
  bool parseFeedback(const canfd_frame & frame, MotorFeedback & feedback) override;

private:
  bool writeInt32Command(uint8_t motor_id, uint8_t command, int32_t value);
  MotorModel modelFor(uint8_t motor_id) const;

  std::map<uint8_t, MotorModel> models_;
};

}  // namespace easyarm_can

#endif  // EASYARM_CAN__DRIVERS__TI5ROBOT_DRIVER_HPP_
