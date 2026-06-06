#include "easyarm_can/driver_factory.hpp"

#include "easyarm_can/drivers/jxservo_driver.hpp"
#include "easyarm_can/drivers/ti5robot_driver.hpp"
#include "easyarm_can/drivers/xhumanoid_driver.hpp"

namespace easyarm_can
{

std::unique_ptr<MotorDriver> createMotorDriver(
  Vendor vendor,
  SocketCanTransport & transport,
  uint8_t host_can_id)
{
  switch (vendor) {
    case Vendor::Jxservo:
      return std::make_unique<JxservoDriver>(transport, host_can_id);
    case Vendor::Ti5robot:
      return std::make_unique<Ti5robotDriver>(transport, host_can_id);
    case Vendor::Xhumanoid:
      return std::make_unique<XhumanoidDriver>(transport, host_can_id);
    case Vendor::Unknown:
      break;
  }
  return nullptr;
}

}  // namespace easyarm_can
