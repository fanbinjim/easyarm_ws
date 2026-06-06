#include "easyarm_can/protocols.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <sstream>
#include <utility>

#include "easyarm_can/encoding.hpp"

namespace easyarm_can
{
namespace
{

class ProtocolBase : public MotorProtocol
{
public:
  ProtocolBase(SocketCanTransport & transport, uint8_t host_can_id)
  : transport_(transport), host_can_id_(host_can_id)
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
  std::string last_error_;
};

class JxservoProtocol final : public ProtocolBase
{
public:
  using ProtocolBase::ProtocolBase;

  Vendor vendor() const override { return Vendor::Jxservo; }

  ProtocolCapabilities capabilities() const override
  {
    ProtocolCapabilities caps;
    caps.hybrid_control = true;
    caps.feedback = true;
    caps.can_fd = true;
    return caps;
  }

  bool configure(uint8_t motor_id, const MotorModel & model) override
  {
    models_[motor_id] = model;
    return true;
  }

  bool clearFault(uint8_t motor_id) override
  {
    return writeControlword(motor_id, 0x0080);
  }

  bool enterHybridMode(uint8_t motor_id) override
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

  bool enableMotor(uint8_t motor_id) override
  {
    return writeControlword(motor_id, 0x000F);
  }

  bool disableMotor(uint8_t motor_id) override
  {
    return writeControlword(motor_id, 0x0000);
  }

  bool sendHybridControl(uint8_t motor_id, const HybridCommand & command) override
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

  bool parseFeedback(const canfd_frame & frame, MotorFeedback & feedback) override
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

private:
  bool writeControlword(uint8_t motor_id, uint16_t value)
  {
    uint8_t data[8] = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    data[4] = static_cast<uint8_t>(value & 0xFFu);
    data[5] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    return sendCan(0x600u + motor_id, data, 8);
  }

  bool writeSdoU8(uint8_t motor_id, uint16_t index, uint8_t subindex, uint8_t value)
  {
    uint8_t data[8] = {};
    data[0] = 0x2F;
    data[1] = static_cast<uint8_t>(index & 0xFFu);
    data[2] = static_cast<uint8_t>((index >> 8) & 0xFFu);
    data[3] = subindex;
    data[4] = value;
    return sendCan(0x600u + motor_id, data, 8);
  }

  MotorModel modelFor(uint8_t motor_id) const
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

  std::map<uint8_t, MotorModel> models_;
};

class Ti5robotProtocol final : public ProtocolBase
{
public:
  using ProtocolBase::ProtocolBase;

  Vendor vendor() const override { return Vendor::Ti5robot; }

  ProtocolCapabilities capabilities() const override
  {
    ProtocolCapabilities caps;
    caps.hybrid_control = false;
    caps.position_control = true;
    caps.velocity_control = true;
    caps.current_control = true;
    caps.feedback = true;
    return caps;
  }

  bool configure(uint8_t motor_id, const MotorModel & model) override
  {
    models_[motor_id] = model;
    return true;
  }

  bool clearFault(uint8_t motor_id) override
  {
    const uint8_t data[1] = {0x0B};
    return sendCan(motor_id, data, 1);
  }

  bool enterHybridMode(uint8_t motor_id) override
  {
    return writeInt32Command(motor_id, 0x71, 0);
  }

  bool enableMotor(uint8_t motor_id) override
  {
    const uint8_t data[1] = {0x01};
    return sendCan(motor_id, data, 1);
  }

  bool disableMotor(uint8_t motor_id) override
  {
    const uint8_t data[1] = {0x02};
    return sendCan(motor_id, data, 1);
  }

  bool sendHybridControl(uint8_t motor_id, const HybridCommand &) override
  {
    (void)motor_id;
    setError(
      "ti5robot PT target frame is not confirmed in repository docs; "
      "enterHybridMode() is available, but sendHybridControl() is intentionally disabled");
    return false;
  }

  bool parseFeedback(const canfd_frame & frame, MotorFeedback & feedback) override
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

private:
  bool writeInt32Command(uint8_t motor_id, uint8_t command, int32_t value)
  {
    uint8_t data[5] = {};
    data[0] = command;
    writeI32Le(&data[1], value);
    return sendCan(motor_id, data, 5);
  }

  MotorModel modelFor(uint8_t motor_id) const
  {
    const auto it = models_.find(motor_id);
    if (it != models_.end()) {
      return it->second;
    }
    MotorModel fallback;
    fallback.vendor = Vendor::Ti5robot;
    return fallback;
  }

  std::map<uint8_t, MotorModel> models_;
};

class XhumanoidProtocol final : public ProtocolBase
{
public:
  using ProtocolBase::ProtocolBase;

  Vendor vendor() const override { return Vendor::Xhumanoid; }

  ProtocolCapabilities capabilities() const override
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

  bool configure(uint8_t motor_id, const MotorModel & model) override
  {
    models_[motor_id] = model;
    counters_[motor_id] = 0;
    return true;
  }

  bool clearFault(uint8_t motor_id) override
  {
    (void)motor_id;
    setError("xhumanoid clearFault frame is not confirmed");
    return false;
  }

  bool enterHybridMode(uint8_t motor_id) override
  {
    (void)motor_id;
    return true;
  }

  bool enableMotor(uint8_t motor_id) override
  {
    const uint8_t data[2] = {0x10, 0x00};
    return sendCanFd(motor_id, data, 2);
  }

  bool disableMotor(uint8_t motor_id) override
  {
    const uint8_t data[2] = {0x10, 0x01};
    return sendCanFd(motor_id, data, 2);
  }

  bool sendHybridControl(uint8_t motor_id, const HybridCommand & command) override
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

  bool parseFeedback(const canfd_frame & frame, MotorFeedback & feedback) override
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

private:
  MotorModel modelFor(uint8_t motor_id) const
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

  std::map<uint8_t, MotorModel> models_;
  std::map<uint8_t, uint8_t> counters_;
};

}  // namespace

std::unique_ptr<MotorProtocol> createProtocol(
  Vendor vendor,
  SocketCanTransport & transport,
  uint8_t host_can_id)
{
  switch (vendor) {
    case Vendor::Jxservo:
      return std::make_unique<JxservoProtocol>(transport, host_can_id);
    case Vendor::Ti5robot:
      return std::make_unique<Ti5robotProtocol>(transport, host_can_id);
    case Vendor::Xhumanoid:
      return std::make_unique<XhumanoidProtocol>(transport, host_can_id);
    case Vendor::Unknown:
      break;
  }
  return nullptr;
}

}  // namespace easyarm_can
