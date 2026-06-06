#include "easyarm_can/drivers/xhumanoid_driver.hpp"

#include <chrono>
#include <cmath>

#include "easyarm_can/encoding.hpp"

namespace easyarm_can
{

Vendor XhumanoidDriver::vendor() const
{
  return Vendor::Xhumanoid;
}

ProtocolCapabilities XhumanoidDriver::capabilities() const
{
  ProtocolCapabilities caps;
  caps.hybrid_control = true;
  caps.position_control = true;
  caps.velocity_control = true;
  caps.current_control = true;
  caps.feedback = true;
  caps.can_fd = true;
  return caps;
}

bool XhumanoidDriver::configure(uint8_t motor_id, const MotorModel & model)
{
  models_[motor_id] = model;
  counters_[motor_id] = 0;
  return true;
}

bool XhumanoidDriver::clearFault(uint8_t motor_id)
{
  (void)motor_id;
  setError("xhumanoid clearFault frame is not confirmed");
  return false;
}

bool XhumanoidDriver::enterHybridMode(uint8_t motor_id)
{
  (void)motor_id;
  return true;
}

bool XhumanoidDriver::enableMotor(uint8_t motor_id)
{
  const uint8_t data[2] = {0x10, 0x00};
  return sendCanFd(motor_id, data, 2);
}

bool XhumanoidDriver::disableMotor(uint8_t motor_id)
{
  const uint8_t data[2] = {0x10, 0x01};
  return sendCanFd(motor_id, data, 2);
}

bool XhumanoidDriver::sendHybridControl(uint8_t motor_id, const HybridCommand & command)
{
  const MotorModel model = modelFor(motor_id);
  const auto & limits = model.limits;
  const uint16_t kp_raw = static_cast<uint16_t>(
    floatToUnsigned(command.kp, limits.kp_min, limits.kp_max, 16));
  const uint16_t kd_raw = static_cast<uint16_t>(
    floatToUnsigned(command.kd, limits.kd_min, limits.kd_max, 16));

  const double bus_position = model.position_unit_degrees_on_bus ?
    radToDeg(command.position_rad) : command.position_rad;
  const double bus_velocity = model.velocity_unit_rpm_on_bus ?
    radPerSecToRpm(command.velocity_rad_s) : command.velocity_rad_s;
  const int16_t torque_raw = static_cast<int16_t>(std::lround(
    clampValue(command.torque_ff_nm * 100.0, -32768.0, 32767.0)));

  uint8_t data[16] = {};
  data[0] = 0x11;
  writeU16Be(&data[1], kp_raw);
  writeU16Be(&data[3], kd_raw);
  writeFloatBe(&data[5], static_cast<float>(bus_position));
  writeFloatBe(&data[9], static_cast<float>(bus_velocity));
  writeI16Be(&data[13], torque_raw);
  data[15] = counters_[motor_id]++;
  return sendCanFd(motor_id, data, 16);
}

bool XhumanoidDriver::parseFeedback(const canfd_frame & frame, MotorFeedback & feedback)
{
  const uint32_t can_id = frame.can_id & CAN_SFF_MASK;
  if (can_id > 0xFFu || frame.len < 15 || frame.data[0] != 0x80) {
    return false;
  }

  const uint8_t motor_id = static_cast<uint8_t>(can_id);
  const MotorModel model = modelFor(motor_id);
  double position = readFloatBe(&frame.data[3]);
  double velocity = readFloatBe(&frame.data[7]);
  if (model.position_unit_degrees_on_bus) {
    while (position < -180.0 || position >= 180.0) {
      position += position < -180.0 ? 360.0 : -360.0;
    }
    position = degToRad(position);
  }
  if (model.velocity_unit_rpm_on_bus) {
    velocity = rpmToRadPerSec(velocity);
  }

  const int16_t current_raw = readI16Be(&frame.data[11]);
  const double current_a = static_cast<double>(current_raw) / 100.0;

  feedback.motor_id = motor_id;
  feedback.position_rad = position;
  feedback.velocity_rad_s = velocity;
  feedback.torque_nm = current_a * model.torque_constant_nm_per_a;
  feedback.temperature_deg_c = static_cast<double>(static_cast<int8_t>(frame.data[13] - 50));
  feedback.fault_code =
    (static_cast<uint32_t>(frame.data[1] & 0x0Fu) << 8) | frame.data[2];
  feedback.enabled = (frame.data[1] & 0x10u) != 0 || ((frame.data[1] >> 4) != 0);
  feedback.is_valid = true;
  feedback.last_update = std::chrono::steady_clock::now();
  return true;
}

MotorModel XhumanoidDriver::modelFor(uint8_t motor_id) const
{
  const auto it = models_.find(motor_id);
  if (it != models_.end()) {
    return it->second;
  }
  MotorModel fallback;
  fallback.vendor = Vendor::Xhumanoid;
  fallback.position_unit_degrees_on_bus = true;
  fallback.velocity_unit_rpm_on_bus = true;
  fallback.torque_ff_raw_int16 = true;
  return fallback;
}

}  // namespace easyarm_can
