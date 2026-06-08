/**
 * @file xhumanoid_driver.hpp
 * @brief XHumanoid 电机协议驱动。
 */

#ifndef EASYARM_CAN__DRIVERS__XHUMANOID_DRIVER_HPP_
#define EASYARM_CAN__DRIVERS__XHUMANOID_DRIVER_HPP_

#include <map>

#include "easyarm_can/motor_driver.hpp"

namespace easyarm_can
{

class XhumanoidDriver final : public MotorDriverBase
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
  bool sendPositionControl(uint8_t motor_id, const PositionCommand & command) override;
  bool parseFeedback(const canfd_frame & frame, MotorFeedback & feedback) override;

private:
  MotorModel modelFor(uint8_t motor_id) const;

  std::map<uint8_t, MotorModel> models_;
  std::map<uint8_t, uint8_t> counters_;
};

}  // namespace easyarm_can

#endif  // EASYARM_CAN__DRIVERS__XHUMANOID_DRIVER_HPP_
