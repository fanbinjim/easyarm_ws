# easyarm_controller

`easyarm_controller` 提供 EasyArm 自定义 `ros2_control` controller。

当前包含：

```text
easyarm_controller/EasyArmDragController
easyarm_controller/EasyArmServoController
```

`EasyArmServoController` 用于 `SERVO` 链路，接收 MoveIt Servo 的 200Hz 流式输出。当前版本固定输出完整关节运控 command 接口：`position + velocity + kp + kd + effort`。其中 `position` 来自上游输入，`velocity` 来自 `JointTrajectory.velocities`，`kp/kd` 来自 controller 参数，`effort` 由 `gravity(q_target)` 计算得到。

`EasyArmDragController` 是 DRAG controller 化的第一阶段原型。它不接收外部运动目标，只读取当前 joint position，并输出 `kp=0`、`velocity=0`、`kd=drag_kd`、`effort=gravity(q)`。默认只 inactive 加载，不替换当前 hardware 内已经真机可用的 `/easyarm/set_mode DRAG`。

## Controller

### EasyArmServoController

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
    command_interfaces:
      - position
      - velocity
      - kp
      - kd
      - effort
    state_interfaces:
      - position
    command_timeout_sec: 0.2
    enable_gravity_compensation: true
    gravity_compensation_scale: 1.0
    kp: 80.0
    kd: 5.0
```

当前 `EasyArmServoController` claim：

```text
Joint1/position
Joint1/velocity
Joint1/kp
Joint1/kd
Joint1/effort
Joint2/position
Joint2/velocity
Joint2/kp
Joint2/kd
Joint2/effort
Joint3/position
Joint3/velocity
Joint3/kp
Joint3/kd
Joint3/effort
Joint4/position
Joint4/velocity
Joint4/kp
Joint4/kd
Joint4/effort
Joint5/position
Joint5/velocity
Joint5/kp
Joint5/kd
Joint5/effort
Joint6/position
Joint6/velocity
Joint6/kp
Joint6/kd
Joint6/effort
```

`command_interfaces` 和 `state_interfaces` 的配置风格参考 `JointTrajectoryController`。当前实现要求：

- `command_interfaces` 必须固定为 `position, velocity, kp, kd, effort`。
- `kp/kd` 是 EasyArm 自定义 command interface，对应电机运控/力位混合控制字段。
- `velocity` 在 `JointTrajectory` 输入带 velocity 时直接写入 hardware command；`Float64MultiArray` position-only 输入时写 `0.0`。
- `effort` 用于写入 controller feedforward effort。
- `state_interfaces` 必须包含 `position`，可为后续扩展预留其他 state interface。

当前 `EasyArmServoController` 读取：

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
/easyarm_servo_controller/joint_positions
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
- 当前 controller 使用 position，解析缓存 velocity / acceleration。
- 输出到 hardware 的 velocity command 来自 MoveIt Servo `JointTrajectory.velocities`；若使用 `Float64MultiArray` position-only 路线则输出 `0.0`。
- acceleration 为后续加速度前馈和完整逆动力学控制预留。
- 如果改回 `std_msgs/Float64MultiArray`，输出 topic 应改为 `/easyarm_servo_controller/joint_positions`，并且必须关闭 velocity / acceleration；当前兼容路径只支持 position-only。

## Dynamics

`EasyArmServoController` 激活时会从 `/robot_description` topic 获取 URDF XML，并通过 `easyarm_dynamics::RobotModel::fromUrdfXml()` 构建 Pinocchio 模型。若 `enable_gravity_compensation=true` 且 3 秒内没有收到 `/robot_description`，controller 激活失败。

当前 feedforward effort 输出为：

```text
HW_IF_EFFORT = gravity(q_target) * gravity_compensation_scale
```

`easyarm_hardware` 会根据 `kp/kd` command interface 是否被 claim 自动选择 full command 来源。`easyarm_servo_controller` active 时使用 controller 输出的 `kp/kd/effort`；切回 `arm_controller` 后自动恢复 hardware 内部兼容逻辑。

`moveit_servo.yaml` 中保留了两种格式的注释，默认使用 `JointTrajectory`：

```yaml
command_out_type: trajectory_msgs/JointTrajectory
command_out_topic: /easyarm_servo_controller/joint_trajectory
```

如果需要临时切到 `Float64MultiArray` position-only 路线，使用：

```yaml
command_out_type: std_msgs/Float64MultiArray
command_out_topic: /easyarm_servo_controller/joint_positions
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

### EasyArmDragController

插件名：

```text
easyarm_controller/EasyArmDragController
```

配置示例：

```yaml
easyarm_drag_controller:
  type: easyarm_controller/EasyArmDragController

easyarm_drag_controller:
  ros__parameters:
    joints:
      - Joint1
      - Joint2
      - Joint3
      - Joint4
      - Joint5
      - Joint6
    command_interfaces:
      - position
      - velocity
      - kp
      - kd
      - effort
    state_interfaces:
      - position
    enable_gravity_compensation: true
    gravity_compensation_scale: 1.0
    kd: 0.10
```

第一阶段原型输出：

```text
position = current joint position
velocity = 0.0
kp = 0.0
kd = kd
effort = gravity(current joint position) * gravity_compensation_scale
```

测试该 controller 时保持 hardware mode 为 `POSITION`，不要调用 `/easyarm/set_mode DRAG`。旧的 hardware `DRAG` 分支仍保留，真机稳定路径不变。

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
ros2 control list_controller_types | grep EasyArmDragController
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
easyarm_drag_controller inactive
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

观察 controller 切换和 full command interface：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

`SpeedJ/SpeedL` 运行期间 `easyarm_servo_controller` 应为 active，`Joint*/position`、`Joint*/velocity`、`Joint*/kp`、`Joint*/kd`、`Joint*/effort` command interface 应被 claimed；输入 timeout 或调用 stop 后应切回 `arm_controller`，这些接口应释放。

停止：

```bash
ros2 run easyarm_app easyarm stop
```

测试 DragController 原型：

```bash
ros2 run easyarm_app easyarm set-mode POSITION
ros2 control switch_controllers --deactivate arm_controller --activate easyarm_drag_controller --strict
ros2 control list_hardware_interfaces
ros2 control switch_controllers --deactivate easyarm_drag_controller --activate arm_controller --strict
```

`easyarm_drag_controller` active 时，`Joint*/position`、`Joint*/velocity`、`Joint*/kp`、`Joint*/kd`、`Joint*/effort` command interface 应被 claimed。该原型只用于验证 controller 层 DRAG 手感，暂不由 `easyarm_motion_server` 自动管理。

## Current Limitations

- 当前版本写 `position + velocity + kp + kd + effort` command interface。
- `EasyArmServoController` 会把 `JointTrajectory.velocities` 写给 hardware；`Float64MultiArray` 输入仍是 position-only，velocity 写 `0.0`。
- JointTrajectory acceleration 已解析和缓存，但暂未用于动力学前馈。
- feedforward effort 当前只包含 gravity，不计算完整 inverse dynamics。
- `MoveJ/MoveL` 仍依赖 `easyarm_hardware` 中现有 gravity compensation。
- 默认 `DRAG` 仍保留在 `easyarm_hardware`，`EasyArmDragController` 只是 inactive 原型。
- 不修改 motor ID、direction、offset、joint limit、CAN 参数、control gain 或 `use_mock_hardware` 默认值。

后续如果要做完整动力学 controller，可以继续使用 `JointTrajectory` 中的 position / velocity / acceleration 生成更完整的 feedforward command。
