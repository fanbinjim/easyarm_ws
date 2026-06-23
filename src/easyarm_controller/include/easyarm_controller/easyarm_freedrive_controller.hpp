#pragma once

#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <easyarm_controller/dynamics_provider.hpp>
#include <easyarm_controller/joint_motion_control_command.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace easyarm_controller
{

/**
 * @brief EasyArm freedrive 控制器第一阶段原型。
 *
 * 该控制器 claim position/velocity/kp/kd/effort command interface，并从 hardware
 * 读取当前关节 position。激活后每个周期输出 kp=0、velocity=0、kd=drag_kd 和
 * gravity(q)，用于验证 FREE_DRIVE 逻辑迁移到 controller 层的可行性。
 */
class EasyArmFreedriveController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> interfaceNames(const std::vector<std::string> & interface_names) const;
  bool configureInterfaces();
  bool readCurrentPositions(std::vector<double> & positions) const;
  bool updateFreedriveCommandFromState();
  void writeCommand();
  size_t commandIndex(size_t joint_index, const std::string & interface_name) const;
  size_t stateIndex(size_t joint_index, const std::string & interface_name) const;

  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_names_;
  std::vector<std::string> state_interface_names_;
  std::vector<JointMotionControlCommand> commands_;
  bool enable_gravity_compensation_{true};
  double gravity_compensation_scale_{1.0};
  double kd_{0.10};
  DynamicsProvider dynamics_provider_;
};

}  // namespace easyarm_controller
