#pragma once

#include <array>
#include <string>
#include <vector>

namespace easyarm_controller
{

inline constexpr const char * kCommandInterfaceKp = "kp";
inline constexpr const char * kCommandInterfaceKd = "kd";

/**
 * @brief EasyArm 关节运控命令。
 *
 * position 单位 rad，velocity 单位 rad/s，kp 单位 Nm/rad，
 * kd 单位 Nm/(rad/s)，effort 单位 Nm。
 */
struct JointMotionControlCommand
{
  double position{0.0};
  double velocity{0.0};
  double kp{80.0};
  double kd{5.0};
  double effort{0.0};
};

const std::array<std::string, 5> & jointMotionControlInterfaceOrder();
std::vector<std::string> jointMotionControlInterfaceVector();

}  // namespace easyarm_controller
