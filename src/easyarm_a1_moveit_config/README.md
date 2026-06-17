# easyarm_a1_moveit_config

`easyarm_a1_moveit_config` 是 EasyArm A1 当前正式 MoveIt 配置包。它保留 H0616
机器人描述和 ros2_control 配置，并作为 `easyarm_a1_bringup` 的配置来源。

本包主要提供 URDF xacro、SRDF、controller 配置、joint limits、kinematics、OMPL
和 Pilz planning pipeline 配置。日常启动推荐使用 `easyarm_a1_bringup`，不要把
`demo.launch.py` 和 `easyarm_a1_bringup` 同时启动。

## Build

```bash
colcon build --packages-select easyarm_a1_moveit_config
source install/setup.bash
```

## 推荐启动方式

Mock：

```bash
ros2 launch easyarm_a1_bringup bringup.launch.py use_mock_hardware:=true
```

真实硬件：

```bash
ros2 launch easyarm_a1_bringup bringup.launch.py
```

真实硬件启动前需要先配置 SocketCAN：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## 直接启动 demo

`demo.launch.py` 是 MoveIt Setup Assistant 生成风格的调试入口，会启动
`robot_state_publisher`、`ros2_control_node`、controllers、`move_group` 和 RViz。
如果使用这个入口，不要同时启动 `easyarm_a1_bringup`。

```bash
ros2 launch easyarm_a1_moveit_config demo.launch.py use_mock_hardware:=true
```

## 关键文件

```text
config/easyarm_a1.urdf.xacro
config/easyarm_a1.ros2_control.xacro
config/easyarm_a1.srdf
config/ros2_controllers.yaml
config/moveit_controllers.yaml
config/joint_limits.yaml
config/pilz_industrial_motion_planner_planning.yaml
config/pilz_cartesian_limits.yaml
```

`use_mock_hardware` 默认值保持为 `false`，直接启动真实硬件前需要确认 CAN 和机械臂处于安全状态。
