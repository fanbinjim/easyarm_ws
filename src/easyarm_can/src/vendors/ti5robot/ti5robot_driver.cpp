#include "easyarm_can/drivers/ti5robot_driver.hpp"

#include <chrono>

#include "easyarm_can/encoding.hpp"

namespace easyarm_can
{

Vendor Ti5robotDriver::vendor() const
{
  return Vendor::Ti5robot;
}

ProtocolCapabilities Ti5robotDriver::capabilities() const
{
  ProtocolCapabilities caps;
  caps.hybrid_control = false;
  caps.position_control = true;
  caps.velocity_control = true;
  caps.current_control = true;
  caps.feedback = true;
  return caps;
}

bool Ti5robotDriver::configure(uint8_t motor_id, const MotorModel & model)
{
  models_[motor_id] = model;
  return true;
}

bool Ti5robotDriver::clearFault(uint8_t motor_id)
{
  const uint8_t data[1] = {0x0B};
  return sendCan(motor_id, data, 1);
}

bool Ti5robotDriver::enterHybridMode(uint8_t motor_id)
{
  return writeInt32Command(motor_id, 0x71, 0);
}

bool Ti5robotDriver::enableMotor(uint8_t motor_id)
{
  const uint8_t data[1] = {0x01};
  return sendCan(motor_id, data, 1);
}

bool Ti5robotDriver::disableMotor(uint8_t motor_id)
{
  const uint8_t data[1] = {0x02};
  return sendCan(motor_id, data, 1);
}

bool Ti5robotDriver::sendHybridControl(uint8_t motor_id, const HybridCommand &)
{
  (void)motor_id;
  setError(
    "ti5robot PT target frame is not confirmed in repository docs; "
    "enterHybridMode() is available, but sendHybridControl() is intentionally disabled");
  return false;
}

bool Ti5robotDriver::parseFeedback(const canfd_frame & frame, MotorFeedback & feedback)
{
  const uint32_t can_id = frame.can_id & CAN_SFF_MASK;
  if (can_id > 0xFFu || frame.len != 8) {
    return false;
  }

  const uint8_t motor_id = static_cast<uint8_t>(can_id);
  const int16_t velocity_raw = readI16Le(&frame.data[0]);
  const int16_t current_ma = readI16Le(&frame.data[2]);
  const int32_t position_cnt = readI32Le(&frame.data[4]);
  const MotorModel model = modelFor(motor_id);

  const double revolutions = model.dual_encoder ?
    static_cast<double>(position_cnt) / 262144.0 :
    static_cast<double>(position_cnt) / 65536.0 / model.gear_ratio;
  const double motor_rpm = static_cast<double>(velocity_raw) * 0.6;
  const double output_rpm = motor_rpm / model.gear_ratio;

  feedback.motor_id = motor_id;
  feedback.position_rad = revolutions * 2.0 * 3.141592653589793;
  feedback.velocity_rad_s = rpmToRadPerSec(output_rpm);
  feedback.torque_nm = static_cast<double>(current_ma) * 0.001 *
    model.torque_constant_nm_per_a;
  feedback.temperature_deg_c = 0.0;
  feedback.fault_code = 0;
  feedback.enabled = true;
  feedback.is_valid = true;
  feedback.last_update = std::chrono::steady_clock::now();
  return true;
}

bool Ti5robotDriver::writeInt32Command(uint8_t motor_id, uint8_t command, int32_t value)
{
  uint8_t data[5] = {};
  data[0] = command;
  writeI32Le(&data[1], value);
  return sendCan(motor_id, data, 5);
}

MotorModel Ti5robotDriver::modelFor(uint8_t motor_id) const
{
  const auto it = models_.find(motor_id);
  if (it != models_.end()) {
    return it->second;
  }
  MotorModel fallback;
  fallback.vendor = Vendor::Ti5robot;
  return fallback;
}

}  // namespace easyarm_can
