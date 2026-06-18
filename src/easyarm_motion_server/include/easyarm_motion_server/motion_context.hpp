#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <string>

namespace easyarm_motion_server
{

struct MotionContext
{
  std::string planning_group{"arm"};
  std::string ee_link{"Link6"};
  std::string planning_frame{"base_link"};
  double default_velocity_scale{0.2};
  double default_acceleration_scale{0.2};
  std::string movej_planner_id{"PTP"};
  std::string movel_planner_id{"LIN"};
  std::string planning_pipeline_id{"pilz_industrial_motion_planner"};
  double joint_state_wait_timeout_sec{5.0};
  double max_joint_state_age_sec{0.5};
  std::array<std::string, 6> joint_names{
    "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"
  };
};

inline std::string normalize_mode(std::string mode)
{
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return mode;
}

inline bool is_valid_mode(const std::string & mode)
{
  return mode == "POSITION" || mode == "IDLE" || mode == "DRAG";
}

}  // namespace easyarm_motion_server
