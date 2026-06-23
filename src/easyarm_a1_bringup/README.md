# easyarm_a1_bringup

`easyarm_a1_bringup` 是 EasyArm A1 的主启动入口。它读取
`easyarm_a1_moveit_config` 中的 URDF、SRDF、controller 和 MoveIt
规划配置，但不会 include `demo.launch.py`。

同一时间不要同时启动：

```bash
ros2 launch easyarm_a1_moveit_config demo.launch.py
```

`demo.launch.py` 会启动另一套 `robot_state_publisher`、`ros2_control_node`、
controllers 和 `move_group`，与本包的 bringup 冲突。

## Build

```bash
colcon build --packages-select easyarm_a1_bringup
source install/setup.bash
```

## Mock Startup

```bash
ros2 launch easyarm_a1_bringup bringup.launch.py use_mock_hardware:=true
```

## Real Hardware Startup

真实硬件启动前先配置 SocketCAN，例如：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

然后启动：

```bash
ros2 launch easyarm_a1_bringup bringup.launch.py
```

## RViz

RViz 默认不启动。调试时可以打开：

```bash
ros2 launch easyarm_a1_bringup bringup.launch.py use_mock_hardware:=true rviz:=true
```

## MoveIt Servo

MoveIt Servo 默认不启动。需要测试 `SpeedJ/SpeedL` 遥操时打开：

```bash
ros2 launch easyarm_a1_bringup bringup.launch.py use_mock_hardware:=true moveit_servo:=true
```

启动后 `arm_controller` 仍保持 active，`easyarm_servo_controller` 和
`easyarm_drag_controller` 只加载为 inactive。
`SpeedJ/SpeedL` 命令进入时由 `easyarm_motion_server` 自动切到
`easyarm_servo_controller`，输入超时或 `/easyarm/stop` 后自动切回
`arm_controller`。

真实硬件上测试前先确认已经切到 `POSITION`，并使用小速度：

```bash
ros2 run easyarm_app easyarm set-mode POSITION
ros2 run easyarm_app easyarm speedj 0 0.05 0 0 0 0 --duration 1.0 --rate 50
ros2 run easyarm_app easyarm speedl 0.01 0 0 0 0 0 --duration 1.0 --rate 50
```

## Checks

检查节点：

```bash
ros2 node list
```

应包含：

```text
/robot_state_publisher
/controller_manager
/move_group
/easyarm_motion_server
/easyarm_hardware_control_mode
```

检查 controllers：

```bash
ros2 control list_controllers
```

应看到：

```text
joint_state_broadcaster active
arm_controller active
easyarm_servo_controller inactive
easyarm_drag_controller inactive
```

检查 motion server：

```bash
ros2 run easyarm_app easyarm get-state
ros2 run easyarm_app easyarm get-joints
ros2 run easyarm_app easyarm get-pose
```

检查 MoveIt Servo：

```bash
ros2 topic info /easyarm/speedj_cmd -v
ros2 topic info /easyarm/speedl_cmd -v
ros2 topic hz /easyarm_servo_controller/joint_trajectory
ros2 topic echo /servo_node/status
```

## Drag Controller Prototype

`easyarm_drag_controller` 是第一阶段 DRAG controller 原型，默认 inactive，不替换
当前已经真机可用的 hardware `DRAG`。

测试时保持 hardware mode 为 `POSITION`，然后手动切 controller：

```bash
ros2 run easyarm_app easyarm set-mode POSITION
ros2 control switch_controllers --deactivate arm_controller --activate easyarm_drag_controller --strict
ros2 control list_hardware_interfaces
ros2 control switch_controllers --deactivate easyarm_drag_controller --activate arm_controller --strict
```

不要用 `/easyarm/set_mode DRAG` 测这个原型；旧 hardware `DRAG` 分支会绕过
controller full command source。

## Safe Shutdown

`safe_shutdown.sh` 会依次停止当前运动、切到 `POSITION`、运动到 ready 位、停用
`arm_controller`、禁用 `EasyArmHardware`，最后关闭 `bringup.launch.py` 或
`demo.launch.py` 进程树。

其中 motion server 相关操作由 `safe_shutdown_motion` 在同一个 ROS 节点里完成，
避免反复启动 `ros2 run easyarm_app easyarm ...`。

真实硬件上执行前确认机械臂运动路径安全：

```bash
ros2 run easyarm_a1_bringup safe_shutdown.sh
```

调试时可以跳过部分步骤：

```bash
SKIP_MOVE_READY=1 ros2 run easyarm_a1_bringup safe_shutdown.sh
SKIP_HARDWARE_DISABLE=1 ros2 run easyarm_a1_bringup safe_shutdown.sh
SKIP_KILL_LAUNCH=1 ros2 run easyarm_a1_bringup safe_shutdown.sh
```

只测试 motion server 相关退出动作时，可以直接运行：

```bash
ros2 run easyarm_a1_bringup safe_shutdown_motion
```
