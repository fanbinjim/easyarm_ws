#include "easyarm_can/drivers/jxservo_driver.hpp"

#include <chrono>

#include "easyarm_can/encoding.hpp"

namespace easyarm_can
{

Vendor JxservoDriver::vendor() const
{
  return Vendor::Jxservo;
}

ProtocolCapabilities JxservoDriver::capabilities() const
{
  ProtocolCapabilities caps;
  caps.hybrid_control = true;
  caps.feedback = true;
  caps.can_fd = true;
  return caps;
}

bool JxservoDriver::configure(uint8_t motor_id, const MotorModel & model)
{
  models_[motor_id] = model;
  return true;
}

bool JxservoDriver::clearFault(uint8_t motor_id)
{
  return writeControlword(motor_id, 0x0080);
}

bool JxservoDriver::enterHybridMode(uint8_t motor_id)
{
  uint8_t data[2] = {0x01, motor_id};
  if (!sendCan(0x000, data, 2)) {
    return false;
  }
  if (!writeSdoU8(motor_id, 0x6060, 0x00, 0xC0)) {
    return false;
  }
  if (!writeControlword(motor_id, 0x0006)) {
    return false;
  }
  if (!writeControlword(motor_id, 0x0007)) {
    return false;
  }
  return writeControlword(motor_id, 0x000F);
}

bool JxservoDriver::enableMotor(uint8_t motor_id)
{
  return writeControlword(motor_id, 0x000F);
}

bool JxservoDriver::disableMotor(uint8_t motor_id)
{
  return writeControlword(motor_id, 0x0000);
}

bool JxservoDriver::sendHybridControl(uint8_t motor_id, const HybridCommand & command)
{
  const MotorModel model = modelFor(motor_id);
  const auto & limits = model.limits;

  const uint16_t q_raw = static_cast<uint16_t>(
    floatToUnsigned(command.position_rad, limits.p_min, limits.p_max, 16));
  const uint16_t dq_raw = static_cast<uint16_t>(
    floatToUnsigned(command.velocity_rad_s, limits.v_min, limits.v_max, 12));
  const uint16_t kp_raw = static_cast<uint16_t>(
    floatToUnsigned(command.kp, limits.kp_min, limits.kp_max, 12));
  const uint16_t kd_raw = static_cast<uint16_t>(
    floatToUnsigned(command.kd, limits.kd_min, limits.kd_max, 12));
  const uint16_t tau_raw = static_cast<uint16_t>(
    floatToUnsigned(command.torque_ff_nm, limits.t_min, limits.t_max, 12));

  uint8_t data[9] = {};
  data[0] = 0xCC;
  data[1] = static_cast<uint8_t>((q_raw >> 8) & 0xFFu);
  data[2] = static_cast<uint8_t>(q_raw & 0xFFu);
  data[3] = static_cast<uint8_t>((dq_raw >> 4) & 0xFFu);
  data[4] = static_cast<uint8_t>(((dq_raw & 0x0Fu) << 4) | ((kp_raw >> 8) & 0x0Fu));
  data[5] = static_cast<uint8_t>(kp_raw & 0xFFu);
  data[6] = static_cast<uint8_t>((kd_raw >> 4) & 0xFFu);
  data[7] = static_cast<uint8_t>(((kd_raw & 0x0Fu) << 4) | ((tau_raw >> 8) & 0x0Fu));
  data[8] = static_cast<uint8_t>(tau_raw & 0xFFu);

  return sendCanFd(0x110u + motor_id, data, 9);
}

bool JxservoDriver::parseFeedback(const canfd_frame & frame, MotorFeedback & feedback)
{
  const uint32_t can_id = frame.can_id & CAN_SFF_MASK;
  if (can_id < 0x300u || can_id > 0x37Fu || frame.len < 12) {
    return false;
  }

  const uint8_t motor_id = static_cast<uint8_t>(can_id - 0x300u);
  const int16_t pos_raw = readI16Be(&frame.data[0]);
  const int16_t rpm_raw = readI16Be(&frame.data[2]);
  const int16_t current_ma = readI16Be(&frame.data[4]);
  const uint16_t error = readU16Be(&frame.data[6]);
  const int16_t temp_raw = readI16Be(&frame.data[8]);
  const uint8_t status = frame.data[11];

  const MotorModel model = modelFor(motor_id);
  feedback.motor_id = motor_id;
  feedback.position_rad = static_cast<double>(pos_raw) / 32768.0 * 3.141592653589793;
  feedback.velocity_rad_s = rpmToRadPerSec(static_cast<double>(rpm_raw)) / model.gear_ratio;
  feedback.torque_nm = static_cast<double>(current_ma) * 0.001 *
    model.torque_constant_nm_per_a;
  feedback.temperature_deg_c = static_cast<double>(temp_raw) * 0.1;
  feedback.fault_code = error;
  feedback.enabled = (status & 0x80u) != 0;
  feedback.is_valid = true;
  feedback.last_update = std::chrono::steady_clock::now();
  return true;
}

bool JxservoDriver::writeControlword(uint8_t motor_id, uint16_t value)
{
  uint8_t data[8] = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  data[4] = static_cast<uint8_t>(value & 0xFFu);
  data[5] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  return sendCan(0x600u + motor_id, data, 8);
}

bool JxservoDriver::writeSdoU8(uint8_t motor_id, uint16_t index, uint8_t subindex, uint8_t value)
{
  uint8_t data[8] = {};
  data[0] = 0x2F;
  data[1] = static_cast<uint8_t>(index & 0xFFu);
  data[2] = static_cast<uint8_t>((index >> 8) & 0xFFu);
  data[3] = subindex;
  data[4] = value;
  return sendCan(0x600u + motor_id, data, 8);
}

MotorModel JxservoDriver::modelFor(uint8_t motor_id) const
{
  const auto it = models_.find(motor_id);
  if (it != models_.end()) {
    return it->second;
  }
  MotorModel fallback;
  fallback.vendor = Vendor::Jxservo;
  fallback.limits.kp_max = 4095.0;
  fallback.limits.kd_max = 255.0;
  return fallback;
}

}  // namespace easyarm_can
