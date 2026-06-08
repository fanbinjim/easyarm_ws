#include "easyarm_can/drivers/xhumanoid_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

#include "easyarm_can/encoding.hpp"

namespace easyarm_can
{
namespace
{

uint32_t floatToUnsignedTrunc(double value, double min_value, double max_value, unsigned bits)
{
  if (bits == 0 || bits >= 32 || max_value <= min_value) {
    return 0;
  }

  const double clamped = clampValue(value, min_value, max_value);
  const uint32_t raw_max = (1u << bits) - 1u;
  const double scaled = (clamped - min_value) * static_cast<double>(raw_max) /
    (max_value - min_value);
  return static_cast<uint32_t>(scaled);
}

}  // namespace

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
  caps.can_fd = is_canfd_;
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
  if (is_canfd_) {
    const uint8_t data[2] = {0x10, 0x01};
    return sendCanFd(motor_id, data, 2);
  }
  (void)motor_id;
  return true;
}

bool XhumanoidDriver::disableMotor(uint8_t motor_id)
{
  if (is_canfd_) {
    const uint8_t data[2] = {0x10, 0x00};
    return sendCanFd(motor_id, data, 2);
  }
  (void)motor_id;
  return true;
}

bool XhumanoidDriver::sendHybridControl(uint8_t motor_id, const HybridCommand & command)
{
  const MotorModel model = modelFor(motor_id);
  const auto & limits = model.limits;

  if (is_canfd_) {
    const uint16_t kp_raw = static_cast<uint16_t>(
      std::lround(clampValue(command.kp, 0.0, 6553.5) * 10.0));
    const uint16_t kd_raw = static_cast<uint16_t>(
      std::lround(clampValue(command.kd, 0.0, 6553.5) * 10.0));
    const int16_t torque_raw = static_cast<int16_t>(
      std::lround(clampValue(command.torque_ff_nm, -32768.0, 32767.0)));

    uint8_t data[16] = {};
    data[0] = 0x11;
    writeU16Be(&data[1], kp_raw);
    writeU16Be(&data[3], kd_raw);
    writeFloatBe(&data[5], static_cast<float>(command.position_rad));
    writeFloatBe(&data[9], static_cast<float>(command.velocity_rad_s));
    writeI16Be(&data[13], torque_raw);
    data[15] = counters_[motor_id]++;
    return sendCanFd(motor_id, data, 16);
  }

  const uint16_t kp_raw = static_cast<uint16_t>(
    floatToUnsignedTrunc(command.kp, limits.kp_min, limits.kp_max, 12));
  const uint16_t kd_raw = static_cast<uint16_t>(
    floatToUnsignedTrunc(command.kd, limits.kd_min, limits.kd_max, 9));
  const uint16_t q_raw = static_cast<uint16_t>(
    floatToUnsignedTrunc(command.position_rad, limits.p_min, limits.p_max, 16));
  const uint16_t dq_raw = static_cast<uint16_t>(
    floatToUnsignedTrunc(command.velocity_rad_s, limits.v_min, limits.v_max, 12));
  const uint16_t tau_raw = static_cast<uint16_t>(
    floatToUnsignedTrunc(command.torque_ff_nm, limits.t_min, limits.t_max, 12));

  uint8_t data[8] = {};
  data[0] = static_cast<uint8_t>((kp_raw >> 7) & 0x1Fu);
  data[1] = static_cast<uint8_t>(((kp_raw & 0x7Fu) << 1) | ((kd_raw >> 8) & 0x01u));
  data[2] = static_cast<uint8_t>(kd_raw & 0xFFu);
  data[3] = static_cast<uint8_t>((q_raw >> 8) & 0xFFu);
  data[4] = static_cast<uint8_t>(q_raw & 0xFFu);
  data[5] = static_cast<uint8_t>((dq_raw >> 4) & 0xFFu);
  data[6] = static_cast<uint8_t>(((dq_raw & 0x0Fu) << 4) | ((tau_raw >> 8) & 0x0Fu));
  data[7] = static_cast<uint8_t>(tau_raw & 0xFFu);
  return sendCan(motor_id, data, 8);
}

bool XhumanoidDriver::parseFeedback(const canfd_frame & frame, MotorFeedback & feedback)
{
  const uint32_t can_id = frame.can_id & CAN_SFF_MASK;
  if (can_id > 0xFFu) {
    return false;
  }

  const uint8_t motor_id = static_cast<uint8_t>(can_id);
  const MotorModel model = modelFor(motor_id);
  const auto & limits = model.limits;

  if (is_canfd_) {
    if (frame.len < 16 || frame.data[0] != 0x80u) {
      return false;
    }

    const uint16_t mode_error = readU16Be(&frame.data[1]);
    const uint8_t mode = static_cast<uint8_t>((mode_error >> 12) & 0x0Fu);
    const uint16_t error = static_cast<uint16_t>(mode_error & 0x0FFFu);
    const double current_a = static_cast<double>(readI16Be(&frame.data[11])) / 100.0;
    const double motor_temp = static_cast<double>(static_cast<int>(frame.data[13]) - 50);
    const double mos_temp = static_cast<double>(static_cast<int>(frame.data[14]) - 50);

    feedback.motor_id = motor_id;
    feedback.position_rad = readFloatBe(&frame.data[3]);
    feedback.velocity_rad_s = readFloatBe(&frame.data[7]);
    feedback.torque_nm = current_a * model.torque_constant_nm_per_a;
    feedback.temperature_deg_c = std::max(motor_temp, mos_temp);
    feedback.fault_code = error;
    feedback.enabled = mode != 0;
    feedback.is_valid = true;
    feedback.last_update = std::chrono::steady_clock::now();
    return true;
  }

  if (frame.len < 8) {
    return false;
  }

  const uint8_t error = frame.data[0] & 0x1Fu;

  if ((frame.data[0] & 0xE0u) == 0xA0u && frame.data[1] == 0x0Au) {
    const uint16_t winding_raw =
      (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
    const uint16_t mos_raw =
      (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
    const double winding_temp = (static_cast<double>(winding_raw) - 50.0) / 2.0;
    const double mos_temp = (static_cast<double>(mos_raw) - 50.0) / 2.0;

    feedback.motor_id = motor_id;
    feedback.temperature_deg_c = std::max(winding_temp, mos_temp);
    feedback.fault_code = error;
    feedback.enabled = true;
    feedback.is_valid = true;
    feedback.last_update = std::chrono::steady_clock::now();
    return true;
  }

  if ((frame.data[0] & 0xE0u) != 0x20u) {
    return false;
  }

  double position = 0.0;
  double velocity = 0.0;
  double current_a = 0.0;

  if (model.reducer_type == ReducerType::Planetary) {
    const uint32_t position_raw =
      (static_cast<uint32_t>(frame.data[1]) << 8) | frame.data[2];
    const uint32_t velocity_raw =
      (static_cast<uint32_t>(frame.data[3]) << 4) | ((frame.data[4] >> 4) & 0x0Fu);
    const uint32_t current_raw =
      (static_cast<uint32_t>(frame.data[4] & 0x0Fu) << 8) | frame.data[5];
    position = unsignedToFloat(position_raw, limits.p_min, limits.p_max, 16);
    velocity = unsignedToFloat(velocity_raw, limits.v_min, limits.v_max, 12);
    current_a = unsignedToFloat(current_raw, -200.0, 200.0, 12);
    feedback.temperature_deg_c = static_cast<double>(static_cast<int>(frame.data[7]) - 50) / 2.0;
  } else {
    const uint32_t position_raw =
      (static_cast<uint32_t>(frame.data[1]) << 16) |
      (static_cast<uint32_t>(frame.data[2]) << 8) |
      frame.data[3];
    const uint32_t velocity_raw =
      (static_cast<uint32_t>(frame.data[4]) << 8) | frame.data[5];
    const uint32_t current_raw =
      (static_cast<uint32_t>(frame.data[6]) << 8) | frame.data[7];
    position = unsignedToFloat(position_raw, limits.p_min, limits.p_max, 24);
    velocity = unsignedToFloat(velocity_raw, limits.v_min, limits.v_max, 16);
    current_a = unsignedToFloat(current_raw, -200.0, 200.0, 16);
    feedback.temperature_deg_c = 0.0;
  }

  feedback.motor_id = motor_id;
  feedback.position_rad = position;
  feedback.velocity_rad_s = velocity;
  feedback.torque_nm = current_a * model.torque_constant_nm_per_a;
  feedback.fault_code = error;
  feedback.enabled = true;
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
  fallback.limits.p_min = -6.28;
  fallback.limits.p_max = 6.28;
  fallback.limits.v_min = -21.0;
  fallback.limits.v_max = 21.0;
  fallback.limits.t_min = -300.0;
  fallback.limits.t_max = 300.0;
  fallback.limits.kp_min = 0.0;
  fallback.limits.kp_max = 2000.0;
  fallback.limits.kd_min = 0.0;
  fallback.limits.kd_max = 300.0;
  fallback.reducer_type = ReducerType::Harmonic;
  return fallback;
}

}  // namespace easyarm_can
