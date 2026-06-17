# easyarm_interfaces

EasyArm 上层运动服务使用的 ROS 2 接口包。该包只定义 action/srv，不包含运动规划、硬件控制或应用逻辑。

## 接口

### MoveJ

```text
/easyarm/movej
easyarm_interfaces/action/MoveJ
```

目标字段：

- `joints`：6 个关节角，单位 `rad`，顺序固定为 `Joint1` 到 `Joint6`。
- `velocity_scale`：速度比例，`<= 0` 时由服务端使用默认值。
- `acceleration_scale`：加速度比例，`<= 0` 时由服务端使用默认值。
- `execute`：`true` 表示规划并执行，`false` 表示只规划。

### MoveL

```text
/easyarm/movel
easyarm_interfaces/action/MoveL
```

目标字段：

- `target_pose`：末端 `Link6` 目标位姿。
- `velocity_scale`：速度比例，`<= 0` 时由服务端使用默认值。
- `acceleration_scale`：加速度比例，`<= 0` 时由服务端使用默认值。
- `execute`：`true` 表示规划并执行，`false` 表示只规划。

`target_pose.header.frame_id` 为空时，服务端默认使用 `base_link`。

### Services

```text
/easyarm/set_mode   easyarm_interfaces/srv/SetMode
/easyarm/stop       easyarm_interfaces/srv/Stop
/easyarm/get_state  easyarm_interfaces/srv/GetState
/easyarm/get_joints easyarm_interfaces/srv/GetJoints
/easyarm/get_pose   easyarm_interfaces/srv/GetPose
```

`SetMode` 支持：

```text
POSITION
IDLE
DRAG
```

## 构建和检查

```bash
colcon build --packages-select easyarm_interfaces
source install/setup.bash
ros2 interface show easyarm_interfaces/action/MoveJ
ros2 interface show easyarm_interfaces/action/MoveL
ros2 interface show easyarm_interfaces/srv/SetMode
ros2 interface show easyarm_interfaces/srv/GetJoints
ros2 interface show easyarm_interfaces/srv/GetPose
```
