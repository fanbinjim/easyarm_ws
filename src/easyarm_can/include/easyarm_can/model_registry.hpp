/**
 * @file model_registry.hpp
 * @brief 电机型号注册表。
 */

#ifndef EASYARM_CAN__MODEL_REGISTRY_HPP_
#define EASYARM_CAN__MODEL_REGISTRY_HPP_

#include <string>
#include <vector>

#include "easyarm_can/types.hpp"

namespace easyarm_can
{

/**
 * @brief 查找电机型号。
 * @param name 形如 "jxservo_default"、"ti5robot_pro2"、"xhumanoid_55p_35"。
 * @param model 输出型号配置。
 * @return 找到返回 true。
 */
bool lookupMotorModel(const std::string & name, MotorModel & model);

/**
 * @brief 返回内置型号名称列表。
 */
std::vector<std::string> builtinMotorModels();

}  // namespace easyarm_can

#endif  // EASYARM_CAN__MODEL_REGISTRY_HPP_
