# EasyArmServoController 第一版实施计划

## Summary

新增 `easyarm_controller` 包，实现 `easyarm_controller/EasyArmServoController`，替代当前 `position_controllers/JointGroupPositionController`。新包必须用 `ros2 pkg create` 创建，并把本计划保存到 `src/easyarm_controller/PLAN.md`。

第一版只做 position passthrough：MoveIt Servo 默认输出 `trajectory_msgs/JointTrajectory`，controller 解析 `positions / velocities / accelerations`，但当前只把 `positions` 写给 hardware。controller 同时保留 `std_msgs/Float64MultiArray` 输入兼容；该输入严格解释为 position-only。`easyarm_hardware` 不修改，gravity compensation 继续保留在 hardware 内。

## Key Changes

- 创建包：
  ```bash
  ros2 pkg create easyarm_controller \
    --build-type ament_cmake \
    --destination-directory src \
    --dependencies controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools std_msgs trajectory_msgs
  ```

- 新增文件：
  ```text
  src/easyarm_controller/PLAN.md
  src/easyarm_controller/include/easyarm_controller/easyarm_servo_controller.hpp
  src/easyarm_controller/src/easyarm_servo_controller.cpp
  src/easyarm_controller/easyarm_controller_plugins.xml
  ```

- controller 插件：
  ```text
  plugin name: easyarm_controller/EasyArmServoController
  class: easyarm_controller::EasyArmServoController
  base: controller_interface::ControllerInterface
  ```

- controller 参数：
  ```yaml
  joints: [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6]
  command_timeout_sec: 0.2
  ```

- controller 行为：
  - claim 每个 joint 的 `position` command interface。
  - 读取每个 joint 的 `position` state interface。
  - 订阅 `~/position_commands`，类型 `std_msgs/msg/Float64MultiArray`，只接受 `N = joints.size()` 个 position。
  - 订阅 `~/joint_trajectory`，类型 `trajectory_msgs/msg/JointTrajectory`。
  - `JointTrajectory` 输入根据 `joint_names` 映射到 controller 的 joints 顺序。
  - 第一版只使用 position 写 command interface。
  - velocity / acceleration 会解析和缓存，但暂不写给 hardware。
  - 无效长度、缺失关节、非有限数值会 warn，并保持上一条有效 command。
  - `on_activate()` 用当前 state position 初始化 hold command。
  - timeout 后继续写 hold position，不清空为 0。
  - `on_deactivate()` 清空 realtime command buffer。

- MoveIt Servo 配置：
  ```yaml
  command_out_type: trajectory_msgs/JointTrajectory
  command_out_topic: /easyarm_servo_controller/joint_trajectory
  publish_joint_positions: true
  publish_joint_velocities: true
  publish_joint_accelerations: true
  publish_period: 0.005
  ```

  兼容输入：

  ```text
  /easyarm_servo_controller/position_commands
    std_msgs/Float64MultiArray
    position-only，长度必须为 N
  ```

- ros2_control 配置：
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

- bringup 更新：
  - spawner 从 `servo_position_controller` 改为 `easyarm_servo_controller`
  - 仍然 `--inactive`
  - `arm_controller` 默认 active 不变

- motion server 更新：
  - `MoveItServoExecutor` controller 名参数化：
    ```text
    servo_controller_name=easyarm_servo_controller
    trajectory_controller_name=arm_controller
    ```
  - 进入 SERVO 激活 `easyarm_servo_controller`，退出切回 `arm_controller`。
  - `/easyarm/speedj_cmd`、`/easyarm/speedl_cmd` 不变。

## Test Plan

- 构建：
  ```bash
  colcon build --packages-select easyarm_controller easyarm_a1_moveit_config easyarm_a1_bringup easyarm_motion_server easyarm_app
  ```

- 插件检查：
  ```bash
  ros2 control list_controller_types | grep EasyArmServoController
  ```

- mock 启动：
  ```bash
  ros2 launch easyarm_a1_bringup bringup.launch.py use_mock_hardware:=true moveit_servo:=true
  ```

- 初始 controller 状态期望：
  ```text
  joint_state_broadcaster active
  arm_controller active
  easyarm_servo_controller inactive
  ```

- Servo 输出检查：
  ```bash
  ros2 topic echo /easyarm_servo_controller/joint_trajectory
  ```
  期望每个 point 包含 `positions`，并可包含 `velocities` 和 `accelerations`。

- 功能回归：
  ```bash
  ros2 run easyarm_app easyarm set-mode POSITION
  ros2 run easyarm_app easyarm speedj 0 0.05 0 0 0 0 --duration 1.0 --rate 50
  ros2 run easyarm_app easyarm speedl 0.01 0 0 0 0 0 --duration 1.0 --rate 50
  ros2 run easyarm_app easyarm movej 0 0 2.35619 0.7854 -1.5708 0 --plan-only
  ros2 run easyarm_app easyarm stop
  ```

## Assumptions

- 第一版只替代 JGPC，不实现动力学 controller。
- 不修改 `easyarm_hardware`。
- 不修改 motor ID、direction、offset、joint limit、CAN 参数、control gain 或 `use_mock_hardware` 默认值。
- MoveIt Servo Humble 的 `Float64MultiArray` 输出模式要求 position 或 velocity 二选一；因此默认改用 `JointTrajectory`，`Float64MultiArray` 只作为 position-only 兼容入口。
- hardware 内部 gravity compensation 继续服务当前 MOVE/SERVO position-only 链路。
- 真机测试必须在 mock 通过后进行，并使用小速度、小幅度。
