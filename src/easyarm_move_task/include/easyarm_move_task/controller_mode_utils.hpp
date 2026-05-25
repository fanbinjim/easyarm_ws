#pragma once

#include <chrono>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

inline bool set_controller_mode(
  rclcpp::Node & node, const std::string & mode,
  std::chrono::seconds timeout = std::chrono::seconds(3))
{
  auto client = node.create_client<rcl_interfaces::srv::SetParameters>(
    "/easyarm_hardware_control_mode/set_parameters");
  if (!client->wait_for_service(timeout)) {
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rclcpp::Parameter param("controller_mode", mode);
  request->parameters.push_back(param.to_parameter_msg());

  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    return false;
  }

  auto result = future.get();
  for (const auto & res : result->results) {
    if (!res.successful) {
      return false;
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  return true;
}
