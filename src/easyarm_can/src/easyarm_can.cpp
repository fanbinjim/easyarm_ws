#include "easyarm_can/easyarm_can.hpp"

#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>

#include <linux/can.h>

#include "easyarm_can/driver_factory.hpp"
#include "easyarm_can/model_registry.hpp"
#include "easyarm_can/socket_can_transport.hpp"

namespace easyarm_can
{

class EasyArmCan::Impl
{
public:
  Impl(const std::string & can_interface, uint8_t host_can_id, bool is_canfd)
  : host_can_id_(host_can_id), is_canfd_(is_canfd), transport_(can_interface)
  {
    drivers_[Vendor::Jxservo] =
      createMotorDriver(Vendor::Jxservo, transport_, host_can_id_, is_canfd_);
    drivers_[Vendor::Ti5robot] =
      createMotorDriver(Vendor::Ti5robot, transport_, host_can_id_, is_canfd_);
    drivers_[Vendor::Xhumanoid] =
      createMotorDriver(Vendor::Xhumanoid, transport_, host_can_id_, is_canfd_);
  }

  ~Impl()
  {
    stopReceiveThread();
    transport_.close();
  }

  bool init()
  {
    return transport_.init(is_canfd_);
  }

  void close()
  {
    stopReceiveThread();
    transport_.close();
  }

  bool isConnected() const
  {
    return transport_.isConnected();
  }

  void setVerbose(bool verbose)
  {
    transport_.setVerbose(verbose);
  }

  bool configureMotor(const MotorConfig & config)
  {
    MotorModel model;
    if (!lookupMotorModel(config.model, model)) {
      setError("unknown motor model: " + config.model);
      return false;
    }

    auto driver = driverFor(model.vendor);
    if (!driver) {
      setError("unsupported motor vendor for model: " + config.model);
      return false;
    }

    if (!driver->configure(config.motor_id, model)) {
      setError(driver->lastError());
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    motor_vendors_[config.motor_id] = model.vendor;
    motor_models_[config.motor_id] = model;
    return true;
  }

  bool configureMotors(const std::vector<MotorConfig> & configs)
  {
    for (const auto & config : configs) {
      if (!configureMotor(config)) {
        return false;
      }
    }
    return true;
  }

  bool clearFault(uint8_t motor_id)
  {
    return dispatch(motor_id, [](MotorDriver & driver, uint8_t id) {
      return driver.clearFault(id);
    });
  }

  bool enterHybridMode(uint8_t motor_id)
  {
    return dispatch(motor_id, [](MotorDriver & driver, uint8_t id) {
      return driver.enterHybridMode(id);
    });
  }

  bool enableMotor(uint8_t motor_id)
  {
    return dispatch(motor_id, [](MotorDriver & driver, uint8_t id) {
      return driver.enableMotor(id);
    });
  }

  bool disableMotor(uint8_t motor_id)
  {
    return dispatch(motor_id, [](MotorDriver & driver, uint8_t id) {
      return driver.disableMotor(id);
    });
  }

  bool sendHybridControl(uint8_t motor_id, const HybridCommand & command)
  {
    return dispatch(motor_id, [&command](MotorDriver & driver, uint8_t id) {
      return driver.sendHybridControl(id, command);
    });
  }

  bool sendPositionControl(uint8_t motor_id, const PositionCommand & command)
  {
    return dispatch(motor_id, [&command](MotorDriver & driver, uint8_t id) {
      return driver.sendPositionControl(id, command);
    });
  }

  bool sendVelocityControl(uint8_t motor_id, const VelocityCommand & command)
  {
    return dispatch(motor_id, [&command](MotorDriver & driver, uint8_t id) {
      return driver.sendVelocityControl(id, command);
    });
  }

  MotorFeedback getMotorFeedback(uint8_t motor_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = feedbacks_.find(motor_id);
    if (it != feedbacks_.end()) {
      return it->second;
    }

    MotorFeedback feedback;
    feedback.motor_id = motor_id;
    feedback.is_valid = false;
    return feedback;
  }

  void startReceiveThread()
  {
    if (receive_running_) {
      return;
    }
    receive_running_ = true;
    receive_thread_ = std::thread([this]() {
      receiveLoop();
    });
  }

  void stopReceiveThread()
  {
    if (!receive_running_) {
      return;
    }
    receive_running_ = false;
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
  }

  ProtocolCapabilities capabilities(uint8_t motor_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto vendor_it = motor_vendors_.find(motor_id);
    if (vendor_it == motor_vendors_.end()) {
      return {};
    }

    const auto driver_it = drivers_.find(vendor_it->second);
    if (driver_it == drivers_.end() || !driver_it->second) {
      return {};
    }
    return driver_it->second->capabilities();
  }

  std::string lastError() const
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    if (!last_error_.empty()) {
      return last_error_;
    }
    return transport_.lastError();
  }

private:
  MotorDriver * driverFor(Vendor vendor)
  {
    const auto it = drivers_.find(vendor);
    if (it != drivers_.end() && it->second) {
      return it->second.get();
    }
    return nullptr;
  }

  MotorDriver * driverForMotor(uint8_t motor_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = motor_vendors_.find(motor_id);
    if (it == motor_vendors_.end()) {
      return nullptr;
    }
    return driverFor(it->second);
  }

  template<typename Callback>
  bool dispatch(uint8_t motor_id, Callback callback)
  {
    auto * driver = driverForMotor(motor_id);
    if (!driver) {
      std::ostringstream oss;
      oss << "motor " << static_cast<int>(motor_id) << " is not configured";
      setError(oss.str());
      return false;
    }

    if (!callback(*driver, motor_id)) {
      setError(driver->lastError());
      return false;
    }
    return true;
  }

  void receiveLoop()
  {
    canfd_frame frame;
    while (receive_running_) {
      if (!transport_.receive(frame, 10)) {
        continue;
      }

      const uint32_t can_id = frame.can_id & CAN_SFF_MASK;
      Vendor configured_vendor = Vendor::Unknown;
      bool has_configured_vendor = false;
      if (can_id <= 0xFFu) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto vendor_it = motor_vendors_.find(static_cast<uint8_t>(can_id));
        if (vendor_it != motor_vendors_.end()) {
          configured_vendor = vendor_it->second;
          has_configured_vendor = true;
        }
      }

      if (has_configured_vendor) {
        auto driver = driverFor(configured_vendor);
        MotorFeedback feedback;
        if (driver && driver->parseFeedback(frame, feedback)) {
          std::lock_guard<std::mutex> lock(mutex_);
          mergeFeedback(feedback);
        }
        continue;
      }

      for (auto & item : drivers_) {
        MotorFeedback feedback;
        if (!item.second || !item.second->parseFeedback(frame, feedback)) {
          continue;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        mergeFeedback(feedback);
        break;
      }
    }
  }

  void mergeFeedback(const MotorFeedback & feedback)
  {
    auto & stored = feedbacks_[feedback.motor_id];
    if (stored.is_valid && feedback.position_rad == 0.0 && feedback.velocity_rad_s == 0.0 &&
      feedback.torque_nm == 0.0 && feedback.temperature_deg_c != 0.0)
    {
      stored.temperature_deg_c = feedback.temperature_deg_c;
      stored.fault_code = feedback.fault_code;
      stored.enabled = feedback.enabled;
      stored.is_valid = true;
      stored.last_update = feedback.last_update;
      return;
    }

    const double previous_temperature = stored.temperature_deg_c;
    stored = feedback;
    if (stored.temperature_deg_c == 0.0 && previous_temperature != 0.0) {
      stored.temperature_deg_c = previous_temperature;
    }
  }

  void setError(const std::string & message)
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
  }

  uint8_t host_can_id_;
  bool is_canfd_{false};
  SocketCanTransport transport_;
  mutable std::mutex mutex_;
  mutable std::mutex error_mutex_;
  std::map<Vendor, std::unique_ptr<MotorDriver>> drivers_;
  std::map<uint8_t, Vendor> motor_vendors_;
  std::map<uint8_t, MotorModel> motor_models_;
  std::map<uint8_t, MotorFeedback> feedbacks_;
  std::string last_error_;
  std::atomic<bool> receive_running_{false};
  std::thread receive_thread_;
};

EasyArmCan::EasyArmCan(const std::string & can_interface, uint8_t host_can_id, bool is_canfd)
: impl_(std::make_unique<Impl>(can_interface, host_can_id, is_canfd))
{
}

EasyArmCan::~EasyArmCan() = default;

bool EasyArmCan::init()
{
  return impl_->init();
}

void EasyArmCan::close()
{
  impl_->close();
}

bool EasyArmCan::isConnected() const
{
  return impl_->isConnected();
}

void EasyArmCan::setVerbose(bool verbose)
{
  impl_->setVerbose(verbose);
}

bool EasyArmCan::configureMotor(const MotorConfig & config)
{
  return impl_->configureMotor(config);
}

bool EasyArmCan::configureMotors(const std::vector<MotorConfig> & configs)
{
  return impl_->configureMotors(configs);
}

bool EasyArmCan::clearFault(uint8_t motor_id)
{
  return impl_->clearFault(motor_id);
}

bool EasyArmCan::enterHybridMode(uint8_t motor_id)
{
  return impl_->enterHybridMode(motor_id);
}

bool EasyArmCan::enableMotor(uint8_t motor_id)
{
  return impl_->enableMotor(motor_id);
}

bool EasyArmCan::disableMotor(uint8_t motor_id)
{
  return impl_->disableMotor(motor_id);
}

bool EasyArmCan::sendHybridControl(uint8_t motor_id, const HybridCommand & command)
{
  return impl_->sendHybridControl(motor_id, command);
}

bool EasyArmCan::sendPositionControl(uint8_t motor_id, const PositionCommand & command)
{
  return impl_->sendPositionControl(motor_id, command);
}

bool EasyArmCan::sendVelocityControl(uint8_t motor_id, const VelocityCommand & command)
{
  return impl_->sendVelocityControl(motor_id, command);
}

MotorFeedback EasyArmCan::getMotorFeedback(uint8_t motor_id) const
{
  return impl_->getMotorFeedback(motor_id);
}

void EasyArmCan::startReceiveThread()
{
  impl_->startReceiveThread();
}

void EasyArmCan::stopReceiveThread()
{
  impl_->stopReceiveThread();
}

ProtocolCapabilities EasyArmCan::capabilities(uint8_t motor_id) const
{
  return impl_->capabilities(motor_id);
}

std::string EasyArmCan::lastError() const
{
  return impl_->lastError();
}

}  // namespace easyarm_can
