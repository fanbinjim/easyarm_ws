/**
 * @file driver_factory.hpp
 * @brief 厂商电机驱动工厂。
 */

#ifndef EASYARM_CAN__DRIVER_FACTORY_HPP_
#define EASYARM_CAN__DRIVER_FACTORY_HPP_

#include <memory>

#include "easyarm_can/motor_driver.hpp"

namespace easyarm_can
{

std::unique_ptr<MotorDriver> createMotorDriver(
  Vendor vendor,
  SocketCanTransport & transport,
  uint8_t host_can_id,
  bool is_canfd);

}  // namespace easyarm_can

#endif  // EASYARM_CAN__DRIVER_FACTORY_HPP_
