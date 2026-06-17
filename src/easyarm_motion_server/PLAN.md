# EasyArm Motion Server 实施计划

## Summary

新增三个包，旧的 `easyarm_move_task` 暂不改：

- `easyarm_interfaces`：ROS action/srv 接口定义。
- `easyarm_motion_server`：常驻运动服务层。
- `easyarm_app`：新的上层 app/CLI 层，后续逐步替代 `easyarm_move_task`。

## Interfaces

第一版接口：

- `/easyarm/movej`：`easyarm_interfaces/action/MoveJ`
- `/easyarm/movel`：`easyarm_interfaces/action/MoveL`
- `/easyarm/set_mode`：`easyarm_interfaces/srv/SetMode`
- `/easyarm/stop`：`easyarm_interfaces/srv/Stop`
- `/easyarm/get_state`：`easyarm_interfaces/srv/GetState`

`MoveJ` 使用关节角，单位为 `rad`，顺序固定为 `Joint1` 到 `Joint6`。
`MoveL` 使用 `geometry_msgs/PoseStamped`，空 `frame_id` 时默认使用 `base_link`。

## Motion Server

节点名和可执行程序均为 `easyarm_motion_server`。默认参数：

```text
planning_group=arm
ee_link=Link6
planning_frame=base_link
default_velocity_scale=0.2
default_acceleration_scale=0.2
movej_planner_id=PTP
movel_planner_id=LIN
planning_pipeline_id=pilz_industrial_motion_planner
```

行为：

- `MoveJ` 走 MoveIt/Pilz `PTP`。
- `MoveL` 走 MoveIt/Pilz `LIN`。
- `MoveJ/MoveL` 执行前检查 hardware `controller_mode`，只有 `POSITION` 模式允许执行；`DRAG`/`IDLE` 模式下直接报错，不自动切换。
- `/easyarm/set_mode` 切到 `POSITION` 前先向 `arm_controller/follow_joint_trajectory` 发送当前点 hold trajectory，避免回到旧目标位置。
- 同一时间只允许一个 goal 执行。
- action cancel 和 `/easyarm/stop` 调用 `move_group.stop()`。
- `/easyarm/set_mode` 封装 `/easyarm_hardware_control_mode/set_parameters`。

第一版不封装 ServoJ/ServoL，遥操后续继续接 MoveIt Servo 原生接口。

## App Layer

`easyarm_app` 提供最小 CLI：

```bash
ros2 run easyarm_app easyarm movej 0.0 1.85 2.69 0.96 1.57 0.0
ros2 run easyarm_app easyarm movel 0.25 0.0 0.25 0.0 0.0 0.0 1.0
ros2 run easyarm_app easyarm set-mode POSITION
ros2 run easyarm_app easyarm get-state
ros2 run easyarm_app easyarm stop
```

CLI 只调用 `easyarm_motion_server`，不直接调用 MoveIt、不直接发 trajectory、不直接碰硬件参数服务。

## Test Plan

构建：

```bash
colcon build --packages-select easyarm_interfaces easyarm_a1_moveit_config easyarm_motion_server easyarm_app
```

mock 测试：

```bash
ros2 launch easyarm_a1_moveit_config demo.launch.py use_mock_hardware:=true
ros2 run easyarm_motion_server easyarm_motion_server
```

真实硬件测试从低速小幅 `MoveJ` 开始，确认 `/easyarm/stop` 和 action cancel 有效后，再测试短距离 `MoveL`。
