# xhumanoid_can

XHumanoid 电机 SocketCAN 驱动库骨架，按 `robstride_can` / `jxservo_can` 的架构组织。

## 架构

- 一个 `XhumanoidCanDriver` 实例绑定一个 CAN 接口，例如 `can0`。
- 一个驱动实例只启动一个接收线程。
- 多台电机共用同一个反馈缓存，按 `motor_id` 索引到 `motor_feedbacks_[motor_id]`。
- 驱动库不依赖 ROS 运行时，仅作为 `ament_cmake` C++17 共享库导出。

## 当前状态

- 已实现 SocketCAN 初始化、关闭、发送、接收线程和反馈缓存骨架。
- `enableMotor()`、`disableMotor()` 和 `parseFeedback()` 需要根据 XHumanoid CAN 协议补齐。

## 构建

```bash
colcon build --packages-select xhumanoid_can
```

## SocketCAN

真实硬件测试前需要先配置 CAN 接口，例如：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

不要在未确认硬件处于安全状态时发送使能、运动或零位相关命令。
