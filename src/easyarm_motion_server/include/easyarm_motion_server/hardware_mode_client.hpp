#pragma once

#include <string>

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

namespace easyarm_motion_server
{

class HardwareModeClient
{
public:
  HardwareModeClient(
    rclcpp::Node & node,
    const rclcpp::CallbackGroup::SharedPtr & callback_group);

  bool queryMode(std::string & mode, std::string & message);
  bool setMode(const std::string & mode, std::string & message);

private:
  rclcpp::Node & node_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr mode_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr mode_get_client_;
};

}  // namespace easyarm_motion_server
