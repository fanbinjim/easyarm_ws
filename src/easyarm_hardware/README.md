# easyarm_hardware

## 控制模式切换

`easyarm_hardware` 当前在 `hardware_interface` 内提供临时模式切换参数，后续会将控制逻辑上提到独立 `easyarm_controller`。

可用模式：

- `IDLE`：纯阻尼模式，`kp=0`，`kd=idle_kd`，`velocity=0`，`tau=0`。
- `POSITION`：正常 MoveIt + `JointTrajectoryController` 轨迹跟踪，在位置控制基础上叠加重力补偿。
- `DRAG`：拖拽示教模式，`kp=0`，`kd=drag_kd`，`velocity=0`，`tau=gravity(q) * drag_gravity_scale`。

切换命令（推荐）：

```bash
ros2 run easyarm_move_task switch_controller_mode IDLE
ros2 run easyarm_move_task switch_controller_mode POSITION
ros2 run easyarm_move_task switch_controller_mode DRAG
```

`switch_controller_mode` 在切到 `POSITION` 时会先将当前 `/joint_states` 位置发给 `arm_controller` 作为 hold trajectory，避免切回后机械臂回旧目标。

底层参数切换（备选）：

```bash
ros2 param set /easyarm_hardware_control_mode controller_mode IDLE
ros2 param set /easyarm_hardware_control_mode controller_mode POSITION
ros2 param set /easyarm_hardware_control_mode controller_mode DRAG
```

查询当前请求模式：

```bash
ros2 param get /easyarm_hardware_control_mode controller_mode
```

`controller_mode` 参数表示请求的控制模式；实际切换在下一次 hardware `write()` 中应用。

实机调试建议先进入 `IDLE` 确认纯阻尼手感，再进入 `DRAG`。切回 `POSITION` 时，hardware 会将命令同步到当前关节位置，降低回弹风险。

`DRAG` 需要满足：

- 底层电机模式为 `motion_control`。
- `enable_gravity_compensation=true`。
- `RobotModel` 已经从 `urdf_path` 成功加载。

相关参数在 `src/easyarm_a1_moveit_config/config/EasyARM-A1.ros2_control.xacro`：

```xml
<param name="hardware_control_mode">position</param>
<param name="gravity_compensation_scale">1.0</param>
<param name="idle_kd">4.0</param>
<param name="drag_gravity_scale">0.2</param>
<param name="drag_kd">1.0</param>
<param name="control_torque_limit_scale">0.5</param>
```

## TODO

- 当前 `IDLE`、`POSITION`、`DRAG` 控制逻辑直接写在 `easyarm_hardware` 中，与硬件接口耦合较大，仅作为真机调试和快速验证的临时方案。
- 待拖拽模式、重力补偿和模式切换稳定后，将控制模式管理、重力补偿、阻尼控制上提到独立 `easyarm_controller`。
- 长期目标是让 `easyarm_hardware` 只保留硬件读写、安全限幅和底层状态同步，控制策略由 controller 层负责。
