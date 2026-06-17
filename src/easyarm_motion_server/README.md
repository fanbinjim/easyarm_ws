# easyarm_motion_server

`easyarm_motion_server` 是 EasyArm 的常驻运动服务层。它对外提供 MoveJ、MoveL、模式切换、停止和状态查询接口，内部复用 MoveIt/Pilz、MoveIt 配置和 `easyarm_hardware` 的 `controller_mode` 参数服务。

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
```

`MoveJ` 使用 Pilz `PTP`，`MoveL` 使用 Pilz `LIN`。`MoveJ/MoveL` 不会自动切换硬件模式，只有当前硬件模式已经是 `POSITION` 时才允许规划和执行；如果当前是 `DRAG` 或 `IDLE`，会直接返回失败。

`/easyarm/set_mode` 切到 `POSITION` 时，会先读取当前 `/joint_states`，向 `arm_controller/follow_joint_trajectory` 发送当前点 hold trajectory，然后再设置 `controller_mode=POSITION`，避免从 `DRAG` 回到 `POSITION` 时回到旧目标位置。

第一版不封装 ServoJ/ServoL，遥操后续继续接 MoveIt Servo 原生接口。

## h0616 启动流程

后续默认使用 `easyarm_a1_h0616_moveit_config`。不要裸跑 `ros2 run easyarm_motion_server easyarm_motion_server`，推荐使用本包的 h0616 launch，让 motion server 获得 h0616 的 `robot_description`、SRDF、kinematics 和 Pilz pipeline 参数。

### 构建

```bash
cd ~/easyarm_ws
colcon build --packages-select \
  easyarm_interfaces \
  easyarm_a1_h0616_moveit_config \
  easyarm_motion_server \
  easyarm_app
source install/setup.bash
```

### Mock 测试

终端 1，启动 MoveIt + ros2_control mock：

```bash
cd ~/easyarm_ws
source install/setup.bash
ros2 launch easyarm_a1_h0616_moveit_config demo.launch.py use_mock_hardware:=true
```

终端 2，启动 motion server：

```bash
cd ~/easyarm_ws
source install/setup.bash
ros2 launch easyarm_motion_server h0616.launch.py use_mock_hardware:=true
```

终端 3，查询状态：

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
ros2 launch easyarm_a1_h0616_moveit_config demo.launch.py
```

终端 2：

```bash
cd ~/easyarm_ws
source install/setup.bash
ros2 launch easyarm_motion_server h0616.launch.py
```

终端 3，先查状态，再低速小幅测试：

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

Pilz 需要关节加速度限制。确认 `easyarm_a1_h0616_moveit_config/config/joint_limits.yaml` 中每个关节都是：

```yaml
has_acceleration_limits: true
max_acceleration: 8.0
```

修改后需要重新构建并重启 `demo.launch.py` 和 `h0616.launch.py`。

### mode 显示 UNKNOWN

`/easyarm/get_state` 中的 `mode` 是 motion server 内部缓存。刚启动且还没有通过 motion server 调过 `/easyarm/set_mode` 或 MoveJ/MoveL 触发过模式检查时，可能会显示 `UNKNOWN`。这不代表规划失败。

### 查询当前关节和末端位姿

`/easyarm/get_joints` 返回最近一次 `/joint_states` 缓存。刚启动时如果还没有收到 `/joint_states`，会返回失败。

`/easyarm/get_pose` 使用 TF 查询末端位姿，默认查询 `base_link -> Link6`：

```bash
ros2 run easyarm_app easyarm get-joints
ros2 run easyarm_app easyarm get-pose
ros2 run easyarm_app easyarm get-pose --target-frame base_link --source-frame Link6
```
