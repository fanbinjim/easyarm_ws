# easyarm_a1_moveit_config

EasyARM-A1 的 MoveIt 配置包，`demo.launch.py` 会启动 MoveIt、RViz、`ros2_control_node`
以及默认控制器。

## 启动 Demo

构建并加载 workspace overlay 后运行：

```bash
colcon build --packages-up-to easyarm_a1_moveit_config
source install/setup.bash
ros2 launch easyarm_a1_moveit_config demo.launch.py
```

当前 ros2_control 配置默认使用真实硬件：

- CAN 接口：`can0`
- 硬件插件：`easyarm_hardware/EasyArmHardware`
- `use_mock_hardware=false`

如需使用硬件插件内置的 mock 模式，不连接 CAN，可启动：

```bash
ros2 launch easyarm_a1_moveit_config demo.launch.py use_mock_hardware:=true
```

启动前请确认机械臂处于安全状态，并已配置 SocketCAN：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## 调试日志参数

`demo.launch.py` 支持 `debug_enable` 参数，用来控制
`easyarm_hardware` 的二进制调试日志。

默认关闭：

```bash
ros2 launch easyarm_a1_moveit_config demo.launch.py
```

显式关闭：

```bash
ros2 launch easyarm_a1_moveit_config demo.launch.py debug_enable:=false
```

开启：

```bash
ros2 launch easyarm_a1_moveit_config demo.launch.py debug_enable:=true
```

开启后，硬件插件会在激活时创建调试日志。日志路径和写入统计会打印在
`ros2_control_node` 输出中；解码方式见 `easyarm_hardware` 包的 README。
