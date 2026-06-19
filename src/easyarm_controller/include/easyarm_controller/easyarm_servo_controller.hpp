#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace easyarm_controller
{

/**
 * @brief EasyArm 实时伺服控制器第一版。
 *
 * 该控制器第一版只 claim position command interface。Float64MultiArray
 * 输入被解释为 position-only；JointTrajectory 输入会解析 position、velocity
 * 和 acceleration，但本版只把 position 写给 hardware。
 */
class EasyArmServoController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  using ArrayCommandMsg = std_msgs::msg::Float64MultiArray;
  using TrajectoryCommandMsg = trajectory_msgs::msg::JointTrajectory;
  struct CommandData
  {
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
    bool has_command{false};
    bool has_velocities{false};
    bool has_accelerations{false};
    rclcpp::Time stamp{static_cast<int64_t>(0), RCL_ROS_TIME};
    std::string source;
  };

  std::vector<std::string> interfaceNames(const std::string & interface_name) const;
  bool readHoldPositionFromState();
  void arrayCommandCallback(const ArrayCommandMsg::SharedPtr message);
  void trajectoryCommandCallback(const TrajectoryCommandMsg::SharedPtr message);
  bool parseArrayCommand(
    const ArrayCommandMsg & message,
    CommandData & command) const;
  bool parseTrajectoryCommand(
    const TrajectoryCommandMsg & message,
    CommandData & command) const;
  bool copyTrajectoryField(
    const std::vector<double> & input,
    const std::vector<int> & joint_index_map,
    const char * field_name,
    std::vector<double> & output) const;
  bool commandTimedOut(
    const rclcpp::Time & time,
    const CommandData & command) const;
  void writeHoldCommand();

  std::vector<std::string> joint_names_;
  std::vector<double> hold_positions_;
  std::vector<double> last_velocities_;
  std::vector<double> last_accelerations_;
  bool has_last_velocities_{false};
  bool has_last_accelerations_{false};
  double command_timeout_sec_{0.2};

  rclcpp::Subscription<ArrayCommandMsg>::SharedPtr array_command_subscriber_;
  rclcpp::Subscription<TrajectoryCommandMsg>::SharedPtr trajectory_command_subscriber_;
  realtime_tools::RealtimeBuffer<CommandData> command_buffer_;
};

}  // namespace easyarm_controller
