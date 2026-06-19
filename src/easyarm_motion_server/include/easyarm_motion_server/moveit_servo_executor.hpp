#pragma once

#include <mutex>
#include <optional>
#include <string>

#include <control_msgs/msg/joint_jog.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace easyarm_motion_server
{

class MoveItServoExecutor
{
public:
  MoveItServoExecutor(
    rclcpp::Node & node,
    const rclcpp::CallbackGroup::SharedPtr & callback_group);

  bool isActive() const;
  bool enterServoRuntime(const std::string & task, std::string & message);
  bool exitServoRuntime(std::string & message);
  void stop();
  void forwardSpeedJ(const control_msgs::msg::JointJog & command);
  void forwardSpeedL(const geometry_msgs::msg::TwistStamped & command);
  void update();

private:
  bool switchControllers(
    const std::string & activate,
    const std::string & deactivate,
    std::string & message);
  std::optional<std::string> controllerState(const std::string & controller_name, std::string & message);
  bool startMoveItServo(std::string & message);
  void publishZeroCommands();
  void publishZeroJointJog();
  void publishZeroTwist();
  bool hasTimedOut(const rclcpp::Time & now) const;

  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr speedj_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr speedl_pub_;
  std::string servo_controller_name_;
  std::string trajectory_controller_name_;

  mutable std::mutex mutex_;
  bool servo_runtime_active_{false};
  std::string active_task_;
  rclcpp::Time last_command_time_;
  double servo_command_timeout_sec_{0.2};
  int halt_message_count_{4};
};

}  // namespace easyarm_motion_server
