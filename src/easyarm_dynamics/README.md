# easyarm_dynamics

`easyarm_dynamics` 是 EasyArm 的动力学计算包。当前模块基于 Pinocchio，提供一个轻量的 C++ `RobotModel` 封装，用于从 URDF 加载机械臂模型并计算刚体动力学项。

该包只负责动力学计算，不包含控制策略、硬件接口或 MoveIt 相关逻辑。

## 功能范围

当前已实现：

- 重力项 `gravity(q)`
- 非线性项 `nle(q, qd)`，即科氏/离心项 `C(q, qd)qd` 与重力项 `g(q)` 的和
- 质量矩阵 `massMatrix(q)`
- 逆动力学 `inverseDynamics(q, qd, qdd)`

状态定义：

- `q`: joint positions, 单位 rad
- `qd`: joint velocities, 单位 rad/s
- `qdd`: joint accelerations, 单位 rad/s^2

所有输入输出均使用 `Eigen::VectorXd` 或 `Eigen::MatrixXd`。

## 目录结构

```text
easyarm_dynamics/
  calibration/
    mass_com_calibration.py
  calibration_results/
  include/easyarm_dynamics/
    robot_model.hpp
  src/
    robot_model.cpp
  CMakeLists.txt
  package.xml
```

## 依赖

ROS 2 Humble 下安装 Pinocchio：

```bash
sudo apt-get install -y ros-humble-pinocchio
```

包依赖：

- `ament_cmake`
- `Eigen3`
- `pinocchio`
- `rclpy` / `control_msgs` / `sensor_msgs` / `trajectory_msgs`（质量/质心标定脚本）
- `python3-numpy` / `python3-scipy` / `python3-yaml`（质量/质心标定脚本）

## 编译

在 workspace 根目录执行：

```bash
cd "$WORKSPACE"
colcon build --packages-select easyarm_dynamics
source install/setup.bash
```

## 使用示例

```cpp
#include <Eigen/Core>
#include "easyarm_dynamics/robot_model.hpp"

int main()
{
  easyarm_dynamics::RobotModel model("/path/to/robot.urdf");

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq());
  Eigen::VectorXd qd = Eigen::VectorXd::Zero(model.nv());
  Eigen::VectorXd qdd = Eigen::VectorXd::Zero(model.nv());

  Eigen::VectorXd g = model.gravity(q);
  Eigen::VectorXd bias = model.nle(q, qd);
  Eigen::MatrixXd m = model.massMatrix(q);
  Eigen::VectorXd tau = model.inverseDynamics(q, qd, qdd);

  return 0;
}
```

## API

```cpp
explicit RobotModel(const std::string & urdf_path);

Eigen::VectorXd gravity(const Eigen::VectorXd & q);
Eigen::VectorXd nle(const Eigen::VectorXd & q, const Eigen::VectorXd & qd);
Eigen::MatrixXd massMatrix(const Eigen::VectorXd & q);
Eigen::VectorXd inverseDynamics(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & qd,
  const Eigen::VectorXd & qdd);

Eigen::Index nq() const noexcept;
Eigen::Index nv() const noexcept;
```

## 设计边界

本包只做：

- URDF 到 Pinocchio model 的加载
- 刚体动力学计算
- 面向控制层的 clean API

本包不做：

- impedance control
- reinforcement learning
- mode switch
- ros2_control hardware interface
- MoveIt 配置或规划逻辑
- 电机 ID、方向、限位、零点或 CAN 参数管理

## 后续开发建议

后续可以在本包中继续增加：

- Jacobian 计算接口
- forward kinematics 接口
- frame velocity / acceleration 接口
- centroidal dynamics 接口
- 面向 controller 的动力学缓存层

控制器本身建议放在独立控制包中，通过 `easyarm_dynamics::RobotModel` 获取动力学量。

## 质量/质心标定工具

`calibration/mass_com_calibration.py` 会通过 `/arm_controller/follow_joint_trajectory`
移动机械臂到一组静态姿态，从 `/joint_states` 采集实际关节角和力矩，并用 Pinocchio
RNEA 拟合 `Link2`-`Link6` 的质量和质心。该工具只输出 YAML，不会修改硬件配置。
默认输出目录为 `src/easyarm_dynamics/calibration_results`。
每次保存标定 YAML 时，脚本也会额外输出同名 `_config/` 目录，格式与
`easyarm_description/config/h0617` 一致，可直接作为 H0617 xacro 的 `config_root`。

当前标定约束：

- 优化 `mass`
- 优化 `COM.y` / `COM.z`
- 固定 `COM.x` 为 URDF 中的原始值

默认 `full` 采样使用 `10°` 间隔：

- `Joint2`、`Joint3` 单关节扫描范围为 `-90° ~ 90°`
- `Joint2 + Joint3` 联合扫描时，如果 `Joint2 > 0`，约束 `Joint2 - Joint3 < 90°`
- `Joint2 + Joint3` 联合扫描时，如果 `Joint2 < 0`，约束 `Joint2 - Joint3 > -90°`
- `Joint4 + Joint5` 联合扫描范围为 `-90° ~ 90°` 的任意组合

查看默认采样点：

```bash
ros2 run easyarm_dynamics mass_com_calibration --dry-run-configs
```

真机采集并优化：

```bash
ros2 run easyarm_dynamics mass_com_calibration --mode full --yes
```

中断后继续：

```bash
ros2 run easyarm_dynamics mass_com_calibration --mode full --resume --yes
```

只使用已有 JSONL 数据重新优化：

```bash
ros2 run easyarm_dynamics mass_com_calibration \
  --optimize-only \
  --data-file src/easyarm_dynamics/calibration_results/mass_com_calibration_data.jsonl
```
