#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <easyarm_dynamics/robot_model.hpp>
#include <rclcpp/logger.hpp>

namespace easyarm_controller
{

/**
 * @brief 为 EasyArm controller 提供动力学模型加载和力矩前馈计算。
 *
 * 该内部组件从 /robot_description 获取 URDF XML，并构建 easyarm_dynamics::RobotModel。
 * 当前前馈输出只包含 gravity(q)，单位为 Nm；后续可扩展为完整动力学前馈。
 */
class DynamicsProvider
{
public:
  bool configure(bool enable_feedforward, double gravity_compensation_scale);
  bool initialize(
    const std::vector<std::string> & joint_names,
    const rclcpp::Logger & logger);
  bool computeFeedforwardEffort(
    const std::vector<double> & positions,
    std::vector<double> & efforts,
    const rclcpp::Logger & logger);

private:
  bool waitForRobotDescription(std::string & robot_description, std::string & message) const;

  bool enable_feedforward_{true};
  double gravity_compensation_scale_{1.0};
  size_t joint_count_{0};
  std::unique_ptr<easyarm_dynamics::RobotModel> robot_model_;
  Eigen::VectorXd gravity_positions_;
  Eigen::VectorXd gravity_torques_;
};

}  // namespace easyarm_controller
