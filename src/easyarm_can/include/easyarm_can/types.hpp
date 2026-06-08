/**
 * @file types.hpp
 * @brief EasyArm CAN 抽象层公共类型。
 */

#ifndef EASYARM_CAN__TYPES_HPP_
#define EASYARM_CAN__TYPES_HPP_

#include <chrono>
#include <cstdint>
#include <string>

namespace easyarm_can
{

/**
 * @brief 厂商协议类型。
 */
enum class Vendor
{
  Jxservo,
  Ti5robot,
  Xhumanoid,
  Unknown
};

/**
 * @brief 关节模组减速器类型。
 */
enum class ReducerType
{
  Unknown,
  Planetary,
  Harmonic
};

/**
 * @brief 混合控制后端能力。
 */
struct ProtocolCapabilities
{
  bool hybrid_control{false};
  bool position_control{false};
  bool velocity_control{false};
  bool current_control{false};
  bool feedback{false};
  bool can_fd{false};
};

/**
 * @brief 电机物理和协议映射范围。
 *
 * 单位：位置 rad，速度 rad/s，力矩 Nm，增益按厂商协议解释。
 */
struct MotorLimits
{
  double p_min{-3.141592653589793};
  double p_max{3.141592653589793};
  double v_min{-50.0};
  double v_max{50.0};
  double t_min{-10.0};
  double t_max{10.0};
  double kp_min{0.0};
  double kp_max{500.0};
  double kd_min{0.0};
  double kd_max{5.0};
};

/**
 * @brief 单个电机型号配置。
 *
 * gear_ratio 用于需要在电机端和负载端换算速度的协议；torque_constant_nm_per_a
 * 用于电流和力矩互算。
 */
struct MotorModel
{
  std::string name;
  Vendor vendor{Vendor::Unknown};
  MotorLimits limits{};
  double gear_ratio{1.0};
  double torque_constant_nm_per_a{0.0};
  ReducerType reducer_type{ReducerType::Unknown};
  bool dual_encoder{false};
  bool position_unit_degrees_on_bus{false};
  bool velocity_unit_rpm_on_bus{false};
  bool torque_ff_raw_int16{false};
};

/**
 * @brief 单个电机实例配置。
 */
struct MotorConfig
{
  uint8_t motor_id{0};
  std::string model;
};

/**
 * @brief MIT-like 力位混合控制命令。
 *
 * 单位：position_rad 为 rad，velocity_rad_s 为 rad/s，torque_ff_nm 为 Nm。
 */
struct HybridCommand
{
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double kp{0.0};
  double kd{0.0};
  double torque_ff_nm{0.0};
};

/**
 * @brief 电机反馈数据。
 *
 * 单位：position_rad 为 rad，velocity_rad_s 为 rad/s，torque_nm 为 Nm，
 * temperature_deg_c 为 degC。
 */
struct MotorFeedback
{
  uint8_t motor_id{0};
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double torque_nm{0.0};
  double temperature_deg_c{0.0};
  uint32_t fault_code{0};
  bool enabled{false};
  bool is_valid{false};
  std::chrono::steady_clock::time_point last_update{};
};

}  // namespace easyarm_can

#endif  // EASYARM_CAN__TYPES_HPP_
