#include "easyarm_motion_server/hardware_mode_client.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include <rcl_interfaces/msg/parameter_type.hpp>

#include "easyarm_motion_server/motion_context.hpp"

namespace easyarm_motion_server
{

HardwareModeClient::HardwareModeClient(
  rclcpp::Node & node,
  const rclcpp::CallbackGroup::SharedPtr & callback_group)
: node_(node)
{
  mode_client_ = node_.create_client<rcl_interfaces::srv::SetParameters>(
    "/easyarm_hardware_control_mode/set_parameters",
    rmw_qos_profile_services_default,
    callback_group);

  mode_get_client_ = node_.create_client<rcl_interfaces::srv::GetParameters>(
    "/easyarm_hardware_control_mode/get_parameters",
    rmw_qos_profile_services_default,
    callback_group);
}

bool HardwareModeClient::queryMode(std::string & mode, std::string & message)
{
  if (!mode_get_client_->wait_for_service(std::chrono::seconds(3))) {
    message = "Hardware control mode get_parameters service is not available";
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names.push_back("controller_mode");

  auto future = mode_get_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    message = "Timeout reading hardware controller_mode";
    return false;
  }

  const auto response = future.get();
  if (response->values.empty() ||
    response->values.front().type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
  {
    message = "Hardware controller_mode parameter is not a string";
    return false;
  }

  mode = normalize_mode(response->values.front().string_value);
  if (!is_valid_mode(mode)) {
    message = "Hardware controller_mode has unknown value '" + response->values.front().string_value + "'";
    return false;
  }

  return true;
}

bool HardwareModeClient::setMode(const std::string & mode, std::string & message)
{
  if (!mode_client_->wait_for_service(std::chrono::seconds(3))) {
    message = "Hardware control mode service is not available";
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  request->parameters.push_back(rclcpp::Parameter("controller_mode", mode).to_parameter_msg());

  auto future = mode_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    message = "Timeout setting hardware mode to " + mode;
    return false;
  }

  const auto response = future.get();
  for (const auto & result : response->results) {
    if (!result.successful) {
      message = result.reason.empty() ? "Failed to set hardware mode to " + mode : result.reason;
      return false;
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  message = "Mode set to " + mode;
  return true;
}

}  // namespace easyarm_motion_server
