#include "easyarm_can/model_registry.hpp"

#include <algorithm>
#include <array>

namespace easyarm_can
{
namespace
{

MotorLimits makeLimits(
  double p_abs,
  double v_abs,
  double t_abs,
  double kp_max,
  double kd_max)
{
  MotorLimits limits;
  limits.p_min = -p_abs;
  limits.p_max = p_abs;
  limits.v_min = -v_abs;
  limits.v_max = v_abs;
  limits.t_min = -t_abs;
  limits.t_max = t_abs;
  limits.kp_min = 0.0;
  limits.kp_max = kp_max;
  limits.kd_min = 0.0;
  limits.kd_max = kd_max;
  return limits;
}

MotorModel jxservoDefault()
{
  MotorModel model;
  model.name = "jxservo_default";
  model.vendor = Vendor::Jxservo;
  model.limits = makeLimits(3.141592653589793, 50.0, 10.0, 4095.0, 255.0);
  return model;
}

MotorModel ti5robotPro2()
{
  MotorModel model;
  model.name = "ti5robot_pro2";
  model.vendor = Vendor::Ti5robot;
  model.limits = makeLimits(3.141592653589793, 50.0, 10.0, 65535.0, 65535.0);
  model.gear_ratio = 1.0;
  return model;
}

MotorModel xhumanoidModel(const char * name, double torque_constant)
{
  MotorModel model;
  model.name = name;
  model.vendor = Vendor::Xhumanoid;
  model.limits = makeLimits(6.28, 21.0, 300.0, 2000.0, 300.0);
  model.torque_constant_nm_per_a = torque_constant;
  model.position_unit_degrees_on_bus = false;
  model.velocity_unit_rpm_on_bus = false;
  model.torque_ff_raw_int16 = false;
  return model;
}

std::vector<MotorModel> makeModels()
{
  return {
    jxservoDefault(),
    ti5robotPro2(),
    xhumanoidModel("xhumanoid_55p_35", 2.6),
    xhumanoidModel("xhumanoid_58p_half_hollow", 1.097),
    xhumanoidModel("xhumanoid_58p_hollow", 1.3702),
    xhumanoidModel("xhumanoid_88p_14_3_2_0", 0.9348),
    xhumanoidModel("xhumanoid_88p_22_5_2_0", 1.572),
    xhumanoidModel("xhumanoid_100p_24", 2.436),
    xhumanoidModel("xhumanoid_125p_20", 2.1),
    xhumanoidModel("xhumanoid_150p_16", 2.001),
    xhumanoidModel("xhumanoid_40h_101", 2.16),
    xhumanoidModel("xhumanoid_55h_50", 3.33),
    xhumanoidModel("xhumanoid_55h_100", 5.97),
    xhumanoidModel("xhumanoid_60h_50", 3.25),
    xhumanoidModel("xhumanoid_60h_100", 6.536),
    xhumanoidModel("xhumanoid_70h_50", 3.456),
    xhumanoidModel("xhumanoid_70h_100", 5.632),
    xhumanoidModel("xhumanoid_80h_51", 3.207),
    xhumanoidModel("xhumanoid_80h_100", 6.14),
  };
}

const std::vector<MotorModel> & models()
{
  static const std::vector<MotorModel> value = makeModels();
  return value;
}

}  // namespace

bool lookupMotorModel(const std::string & name, MotorModel & model)
{
  const auto & all_models = models();
  const auto it = std::find_if(
    all_models.begin(),
    all_models.end(),
    [&name](const MotorModel & candidate) {
      return candidate.name == name;
    });

  if (it == all_models.end()) {
    return false;
  }

  model = *it;
  return true;
}

std::vector<std::string> builtinMotorModels()
{
  std::vector<std::string> names;
  names.reserve(models().size());
  for (const auto & model : models()) {
    names.push_back(model.name);
  }
  return names;
}

}  // namespace easyarm_can
