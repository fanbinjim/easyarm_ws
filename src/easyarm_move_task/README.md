# easyarm_move_task

上层控制任务工具，依赖 MoveIt + `easyarm_hardware`。

## 工具

### move_to_ready

将机械臂运动到预定义的 "ready" 姿态。执行前自动将 hardware 切换到 `POSITION` 模式。

```bash
ros2 run easyarm_move_task move_to_ready
```

### switch_controller_mode

切换 hardware 控制模式。切到 `POSITION` 时会先将当前 `/joint_states` 发给 `arm_controller` 作为 hold trajectory，避免回弹。

```bash
ros2 run easyarm_move_task switch_controller_mode IDLE
ros2 run easyarm_move_task switch_controller_mode POSITION
ros2 run easyarm_move_task switch_controller_mode DRAG
```

模式说明：

| 模式 | kp | kd | velocity | torque |
|------|----|----|----------|--------|
| IDLE | 0 | idle_kd | 0 | 0 |
| POSITION | 10.0 | 5.0 | velocity ff | gravity(q) * scale |
| DRAG | 0 | drag_kd | 0 | gravity(q) * scale |

### easyarm_record

位姿录入工具。启动后自动将 hardware 切换到 `DRAG` 模式，按空格开始以 200Hz 录制 `Joint1`-`Joint6` 的关节角和末端位姿，再次按空格结束录制并保存 JSON。录制结束后保持 `DRAG` 模式。

```bash
ros2 run easyarm_move_task easyarm_record
ros2 run easyarm_move_task easyarm_record my_pose_record.json
```

默认输出到当前 workspace 的 `data/path_record/<YYYYMMDD>/<HH-MM-SS>.json`，例如 `data/path_record/20260525/14-30-05.json`。关节角单位为 `rad`，末端平移单位为 `m`，旋转为四元数 `[x, y, z, w]`。

默认通过 TF 记录 `base_link -> Link6`，可通过参数修改：

```bash
ros2 run easyarm_move_task easyarm_record my_pose_record.json --ros-args -p base_frame:=base_link -p end_effector_frame:=Link6
```

JSON 顶层 `ee_frame` 保存末端 frame 名称。每个样本中的 `joints` 数组按 `joint_names` 顺序保存 6 个关节角，`ee_pose` 保存末端位姿。

### easyarm_replay_rviz.py

将 `easyarm_record` 录制的 JSON 转换为 MoveIt `DisplayTrajectory`，发布到 `/display_planned_path`，用于在 MoveIt RViz 里显示黄色规划轨迹。该工具不切换 hardware 模式，不控制真实机械臂。

```bash
ros2 run easyarm_move_task easyarm_replay_rviz data/path_record/20260525/14-30-05.json
```

使用前先打开 MoveIt RViz，例如：

```bash
ros2 launch easyarm_a1_moveit_config demo.launch.py
```

### easyarm_playback

真机交互播放 `easyarm_record` 录制的 JSON。启动后先发送当前点 hold trajectory，再切换到 `POSITION`，慢速运动到录制起始点并默认暂停。该工具会控制真实机械臂，运行前必须确认硬件和工作空间安全。

```bash
ros2 run easyarm_move_task easyarm_playback data/path_record/20260525/19-50-13.json
```

默认需要输入 `yes` 确认后才会执行。可选参数示例：

```bash
# 将路径替换为实际录制文件
ros2 run easyarm_move_task easyarm_playback data/path_record/20260525/19-50-13.json --ros-args -p speed_scale:=0.5 -p approach_velocity:=0.2 -p max_playback_velocity:=6.0 -p autorepeat:=true -p playback_start_delay:=0.01

ros2 run easyarm_move_task easyarm_playback data/path_record/20260525/19-50-13.json --ros-args -p speed_scale:=1.0 -p approach_velocity:=1.0 -p max_playback_velocity:=6.0 -p autorepeat:=false -p playback_start_delay:=0.3

ros2 run easyarm_move_task easyarm_playback data/path_record/20260615/13-54-47.json --ros-args -p speed_scale:=0.5 -p approach_velocity:=0.2 -p max_playback_velocity:=6.0 -p autorepeat:=false -p playback_start_delay:=0.3

```

播放结束后自动回到起始点并重复播放：

```bash
ros2 run easyarm_move_task easyarm_playback data/path_record/20260525/19-50-13.json --ros-args -p autorepeat:=true
```

按键控制：

| 按键 | 行为 |
|------|------|
| 空格 | 播放 / 暂停 |
| 右方向键 | 暂停时运动到下一个采样点 |
| 左方向键 | 暂停时运动到上一个采样点 |
| q | hold 当前点并退出 |

播放结束后暂停在最后一个点，保持 `POSITION` 模式。

### safe_shutdown_demo.sh

安全关机脚本：先 `move_to_ready`，再停 `arm_controller`、禁用 hardware，最后终止 demo 进程树。

```bash
./install/easyarm_move_task/lib/easyarm_move_task/safe_shutdown_demo.sh
```

推荐通过根目录 wrapper 调用：

```bash
./scripts/safe_shutdown_easyarm.sh
```

## 相关参数

控制模式切换的参数定义在 `src/easyarm_a1_moveit_config/config/EasyARM-A1.ros2_control.xacro`：

```xml
<param name="hardware_control_mode">position</param>
<param name="gravity_compensation_scale">1.0</param>
<param name="idle_kd">4.0</param>
<param name="drag_gravity_scale">1.0</param>
<param name="drag_kd">1.0</param>
<param name="control_torque_limit_scale">0.5</param>
```

运行时也可通过 parameter 临时调整：

```bash
ros2 param set /easyarm_hardware_control_mode controller_mode DRAG
ros2 param set /easyarm_hardware_control_mode drag_kd 2.0
ros2 param get /easyarm_hardware_control_mode controller_mode
```

## 架构说明

当前 `IDLE`、`POSITION`、`DRAG` 控制逻辑临时耦合在 `easyarm_hardware` 中。`easyarm_move_task` 只负责上层任务调度和模式切换命令，不直接操作 CAN 或执行电机控制。
