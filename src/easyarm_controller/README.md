# easyarm_controller

`easyarm_controller` 提供 EasyArm 自定义 `ros2_control` controller。

当前第一版只包含：

```text
easyarm_controller/EasyArmServoController
```

该 controller 用于 `SERVO` 链路，接收 MoveIt Servo 的 200Hz 流式输出。第一版只把目标关节位置写给 `hardware_interface` 的 `position` command interface；`velocity` 和 `acceleration` 会被解析和缓存，但暂时不写给硬件。

## Controller

插件名：

```text
easyarm_controller/EasyArmServoController
```

配置示例：

```yaml
easyarm_servo_controller:
  type: easyarm_controller/EasyArmServoController

easyarm_servo_controller:
  ros__parameters:
    joints:
      - Joint1
      - Joint2
      - Joint3
      - Joint4
      - Joint5
      - Joint6
    command_timeout_sec: 0.2
```

当前 controller claim：

```text
Joint1/position
Joint2/position
Joint3/position
Joint4/position
Joint5/position
Joint6/position
```

当前 controller 读取：

```text
Joint1/position
Joint2/position
Joint3/position
Joint4/position
Joint5/position
Joint6/position
```

## Input Topics

controller 支持两个输入 topic。

默认 MoveIt Servo 链路使用：

```text
/easyarm_servo_controller/joint_trajectory
trajectory_msgs/msg/JointTrajectory
```

`JointTrajectory` 输入会按 `joint_names` 映射到 controller 的关节顺序。第一版使用：

```text
points[0].positions
```

并解析缓存：

```text
points[0].velocities
points[0].accelerations
```

兼容输入：

```text
/easyarm_servo_controller/position_commands
std_msgs/msg/Float64MultiArray
```

`Float64MultiArray` 只解释为 position-only，长度必须等于关节数量。它不承载 velocity 或 acceleration 语义。

## MoveIt Servo Config

当前默认配置在：

```text
src/easyarm_a1_moveit_config/config/moveit_servo.yaml
```

关键参数：

```yaml
publish_period: 0.005
command_out_type: trajectory_msgs/JointTrajectory
command_out_topic: /easyarm_servo_controller/joint_trajectory
publish_joint_positions: true
publish_joint_velocities: true
publish_joint_accelerations: true
```

说明：

- `publish_period: 0.005` 对应 200Hz 输出目标。
- 当前 controller 只使用 position。
- velocity / acceleration 为后续速度前馈、加速度前馈和动力学控制预留。
- 如果改回 `std_msgs/Float64MultiArray`，输出 topic 应改为 `/easyarm_servo_controller/position_commands`，并且必须关闭 velocity / acceleration；当前兼容路径只支持 position-only。

`moveit_servo.yaml` 中保留了两种格式的注释，默认使用 `JointTrajectory`：

```yaml
command_out_type: trajectory_msgs/JointTrajectory
command_out_topic: /easyarm_servo_controller/joint_trajectory
```

如果需要临时切到 `Float64MultiArray` position-only 路线，使用：

```yaml
command_out_type: std_msgs/Float64MultiArray
command_out_topic: /easyarm_servo_controller/position_commands
publish_joint_positions: true
publish_joint_velocities: false
publish_joint_accelerations: false
```

## Runtime Flow

正常 `SpeedJ/SpeedL` 链路：

```text
easyarm_app
  -> /easyarm/speedj_cmd 或 /easyarm/speedl_cmd
    -> easyarm_motion_server
      -> /servo_node/delta_joint_cmds 或 /servo_node/delta_twist_cmds
        -> MoveIt Servo
          -> /easyarm_servo_controller/joint_trajectory
            -> easyarm_servo_controller
              -> easyarm_hardware
```

`easyarm_motion_server` 负责在 `arm_controller` 和 `easyarm_servo_controller` 之间切换。不要手动同时激活这两个 controller。

## Build

```bash
cd ~/easyarm_ws
colcon build --packages-select easyarm_controller
source install/setup.bash
```

通常和相关包一起构建：

```bash
colcon build --packages-select \
  easyarm_controller \
  easyarm_a1_moveit_config \
  easyarm_a1_bringup \
  easyarm_motion_server \
  easyarm_app
source install/setup.bash
```

## Test

检查插件：

```bash
ros2 control list_controller_types | grep EasyArmServoController
```

启动 mock：

```bash
ros2 launch easyarm_a1_bringup bringup.launch.py use_mock_hardware:=true moveit_servo:=true
```

检查初始 controller 状态：

```bash
ros2 control list_controllers
```

期望：

```text
joint_state_broadcaster active
arm_controller active
easyarm_servo_controller inactive
```

运行 SpeedJ：

```bash
ros2 run easyarm_app easyarm set-mode POSITION
ros2 run easyarm_app easyarm speedj 0 0.05 0 0 0 0 --duration 1.0 --rate 50
```

观察 MoveIt Servo 输出：

```bash
ros2 topic hz /easyarm_servo_controller/joint_trajectory
ros2 topic echo /easyarm_servo_controller/joint_trajectory
```

观察 controller 切换：

```bash
ros2 control list_controllers
```

`SpeedJ/SpeedL` 运行期间 `easyarm_servo_controller` 应为 active；输入 timeout 或调用 stop 后应切回 `arm_controller`。

停止：

```bash
ros2 run easyarm_app easyarm stop
```

## Current Limitations

- 第一版只写 position command interface。
- velocity / acceleration 已解析和缓存，但暂未用于硬件输出。
- 不计算动力学、effort 或 torque feedforward。
- 不修改 `easyarm_hardware` 中现有 gravity compensation。
- 不修改 motor ID、direction、offset、joint limit、CAN 参数、control gain 或 `use_mock_hardware` 默认值。

后续如果要做完整动力学 controller，可以在 `EasyArmServoController` 内继续接入 `easyarm_dynamics`，使用 `JointTrajectory` 中的 position / velocity / acceleration 生成更完整的 command。
