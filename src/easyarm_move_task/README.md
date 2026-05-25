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
