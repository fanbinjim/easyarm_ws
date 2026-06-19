# easyarm_motion_server

`easyarm_motion_server` 是 EasyArm 的常驻运动服务层。它对外提供 MoveJ、MoveL、SpeedJ、SpeedL、模式切换、停止和状态查询接口，内部复用 MoveIt/Pilz、MoveIt Servo、MoveIt 配置和 `easyarm_hardware` 的 `controller_mode` 参数服务。

旧的 `easyarm_move_task` 暂不迁移，仍作为已有 app/demo 工具保留。

## 接口

```text
/easyarm/movej      easyarm_interfaces/action/MoveJ
/easyarm/movel      easyarm_interfaces/action/MoveL
/easyarm/set_mode   easyarm_interfaces/srv/SetMode
/easyarm/stop       easyarm_interfaces/srv/Stop
/easyarm/get_state  easyarm_interfaces/srv/GetState
/easyarm/get_joints easyarm_interfaces/srv/GetJoints
/easyarm/get_pose   easyarm_interfaces/srv/GetPose
/easyarm/speedj_cmd control_msgs/msg/JointJog
/easyarm/speedl_cmd geometry_msgs/msg/TwistStamped
```

默认参数：

```text
planning_group=arm
ee_link=Link6
planning_frame=base_link
default_velocity_scale=0.2
default_acceleration_scale=0.2
movej_planner_id=PTP
movel_planner_id=LIN
planning_pipeline_id=pilz_industrial_motion_planner
joint_state_wait_timeout=5.0
max_joint_state_age=0.5
```

`MoveJ` 使用 Pilz `PTP`，`MoveL` 使用 Pilz `LIN`。`MoveJ/MoveL` 不会自动切换硬件模式，只有当前硬件模式已经是 `POSITION` 时才允许规划和执行；如果当前是 `DRAG` 或 `IDLE`，会直接返回失败。

执行 MoveJ/MoveL 前会等待 `/joint_states` 包含 6 个关节并且时间戳足够新，避免刚启动时 MoveIt 因当前状态过期而在执行阶段 abort。

`/easyarm/set_mode` 切到 `POSITION` 时，会先读取当前 `/joint_states`，向 `arm_controller/follow_joint_trajectory` 发送当前点 hold trajectory，然后再设置 `controller_mode=POSITION`，避免从 `DRAG` 回到 `POSITION` 时回到旧目标位置。

`SpeedJ/SpeedL` 使用流式 topic 输入。收到 `/easyarm/speedj_cmd` 或 `/easyarm/speedl_cmd` 后，motion server 会在硬件模式为 `POSITION` 时自动切换到 `servo_position_controller`，启动 MoveIt Servo，并转发命令到 `/servo_node/delta_joint_cmds` 或 `/servo_node/delta_twist_cmds`。输入超时或调用 `/easyarm/stop` 后，会发送 zero command 并切回 `arm_controller`。

历史说明：当前 `/easyarm/set_mode` 和 `/easyarm/get_state.mode` 封装的是 `easyarm_hardware` 的 hardware mode，只支持 `POSITION/IDLE/DRAG`。这些不是理想的上层 `MOVE/SERVO/DRAG` control mode 分层。本次保留该历史行为，避免影响已有 DRAG、safe shutdown 和真机安全逻辑。

第一版不封装 ServoJ/ServoL。

## 启动流程

默认通过 `easyarm_a1_bringup` 启动完整运行链路。不要裸跑 `ros2 run easyarm_motion_server easyarm_motion_server`，也不要在 `easyarm_a1_bringup` 已经运行时再启动 `easyarm_motion_server/launch/h0616.launch.py`，否则会出现重复的 `/easyarm/movej` action server。

`h0616.launch.py` 仅作为兼容/调试入口保留，用于已经单独启动 MoveIt 和 ros2_control 时给 motion server 注入 `robot_description`、SRDF、kinematics 和 Pilz pipeline 参数。

### 构建

```bash
cd ~/easyarm_ws
colcon build --packages-select \
  easyarm_interfaces \
  easyarm_a1_moveit_config \
  easyarm_a1_bringup \
  easyarm_motion_server \
  easyarm_app
source install/setup.bash
```

### Mock 测试

终端 1，启动 bringup：

```bash
cd ~/easyarm_ws
source install/setup.bash
ros2 launch easyarm_a1_bringup bringup.launch.py use_mock_hardware:=true moveit_servo:=true
```

终端 2，查询状态：

```bash
cd ~/easyarm_ws
source install/setup.bash
ros2 run easyarm_app easyarm get-state
ros2 run easyarm_app easyarm get-joints
ros2 run easyarm_app easyarm get-pose
```

如果刚才进入过 `DRAG` 或 `IDLE`，先显式切回 `POSITION`：

```bash
ros2 run easyarm_app easyarm set-mode POSITION
```

先只规划：

```bash
ros2 run easyarm_app easyarm movej 0.0 1.85 2.69 0.96 1.57 0.0 --plan-only
```

规划成功后再执行：

```bash
ros2 run easyarm_app easyarm movej 0.0 1.85 2.69 0.96 1.57 0.0 \
  --velocity-scale 0.2 \
  --acceleration-scale 0.2
```

MoveL 示例：

```bash
ros2 run easyarm_app easyarm movel 0.25 0.0 0.25 0.0 0.0 0.0 1.0 \
  --velocity-scale 0.1 \
  --acceleration-scale 0.1 \
  --plan-only
```

SpeedJ 示例：

```bash
ros2 run easyarm_app easyarm speedj 0 0.05 0 0 0 0 --duration 1.0 --rate 50
```

SpeedL 示例：

```bash
ros2 run easyarm_app easyarm speedl 0.01 0 0 0 0 0 --duration 1.0 --rate 50
```

停止：

```bash
ros2 run easyarm_app easyarm stop
```

### 真实硬件测试

先配置 CAN：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

终端 1：

```bash
cd ~/easyarm_ws
source install/setup.bash
ros2 launch easyarm_a1_bringup bringup.launch.py
```

终端 2，先查状态，再低速小幅测试：

```bash
cd ~/easyarm_ws
source install/setup.bash
ros2 run easyarm_app easyarm get-state
ros2 run easyarm_app easyarm movej 0.0 1.85 2.69 0.96 1.57 0.0 \
  --velocity-scale 0.05 \
  --acceleration-scale 0.05
```

真实硬件第一次测试建议先使用 `--plan-only`，确认规划成功后再执行。执行前确认机械臂处于安全状态，急停或断电手段可用。

## 常见问题

### MoveJ planning failed，日志里有 acceleration limit not set

Pilz 需要关节加速度限制。确认 `easyarm_a1_moveit_config/config/joint_limits.yaml` 中每个关节都是：

```yaml
has_acceleration_limits: true
max_acceleration: 8.0
```

修改后需要重新构建并重启 `easyarm_a1_bringup`。

### mode 查询失败

`/easyarm/get_state` 会主动查询 `/easyarm_hardware_control_mode/get_parameters` 并返回当前硬件模式。刚启动时如果硬件模式参数服务还不可用，会返回失败信息，并保留 motion server 内部缓存模式。

### 查询当前关节和末端位姿

`/easyarm/get_joints` 返回最近一次 `/joint_states` 缓存。刚启动时如果还没有收到 `/joint_states`，会返回失败。

`/easyarm/get_pose` 使用 TF 查询末端位姿，默认查询 `base_link -> Link6`：

```bash
ros2 run easyarm_app easyarm get-joints
ros2 run easyarm_app easyarm get-pose
ros2 run easyarm_app easyarm get-pose --target-frame base_link --source-frame Link6
```
