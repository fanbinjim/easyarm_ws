#include "easyarm_controller/joint_motion_control_command.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace easyarm_controller
{

const std::array<std::string, 5> & jointMotionControlInterfaceOrder()
{
  static const std::array<std::string, 5> interfaces{
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    kCommandInterfaceKp,
    kCommandInterfaceKd,
    hardware_interface::HW_IF_EFFORT};
  return interfaces;
}

std::vector<std::string> jointMotionControlInterfaceVector()
{
  const auto & interfaces = jointMotionControlInterfaceOrder();
  return std::vector<std::string>(interfaces.begin(), interfaces.end());
}

}  // namespace easyarm_controller
