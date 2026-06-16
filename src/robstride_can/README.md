# robstride_can

`robstride_can` 是一个 ROS 2 `ament_cmake` 库包，用于封装 Robstride 电机的 CAN 通信逻辑。它基于 Linux SocketCAN，实现了 Robstride 私有 CAN 协议，并对外提供一个可复用的 C++ 驱动接口。

这个包的定位是底层 CAN 驱动库，可以被 `easyarm_hardware` 这类硬件接口包依赖使用。

编译后会生成动态库：

```text
librobstride_can.so
```

## 功能

- 基于 SocketCAN 的 Robstride CAN 通信驱动。
- 支持 `RS00`、`EL05`、`RS05` 三种电机参数范围。
- 支持电机使能、失能、清故障、设置机械零位。
- 支持运控模式指令发送。
- 支持写入运行模式、CSP 位置指令、速度上限等参数。
- 内置接收线程，用于接收 Type 2 电机反馈帧。
- 缓存电机反馈数据，包括位置、速度、力矩、温度、模式状态、故障码。
- 提供 CAN 发送重试次数和失败次数统计，便于排查时序或总线问题。

## 支持的电机类型

| 类型 | 速度范围 | 力矩范围 | 典型用途 |
| --- | --- | --- | --- |
| `RS00` | +/-33 rad/s | +/-14 Nm | 较大关节 |
| `EL05` | +/-50 rad/s | +/-6 Nm | 较小关节 |
| `RS05` | +/-50 rad/s | +/-5.5 Nm | 较小关节 |

所有电机类型使用相同的位置范围：`[-12.57, 12.57] rad`。

增益范围也相同：

- `Kp`: `[0, 500]`
- `Kd`: `[0, 5]`

## 编译

在 workspace 根目录执行：

```bash
colcon build --packages-select robstride_can
```

编译完成后 source 工作空间：

```bash
source install/setup.bash
```

编译成功后，动态库会安装到：

```text
install/robstride_can/lib/librobstride_can.so
```

同时会安装单电机测试程序：

```text
install/robstride_can/lib/robstride_can/single_motor_demo
install/robstride_can/lib/robstride_can/discover_motors
install/robstride_can/lib/robstride_can/set_zero_pos
```

## CAN 接口准备

驱动要求系统中已经存在并启用了 Linux SocketCAN 接口，例如 `can0`。调用 `RobstrideCanDriver::init()` 之前，需要先把 CAN 接口配置好。

示例：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

如果使用 candleLight/gs_usb 固件的 USB-CAN 适配器，需要确认设备已经被内核识别，并暴露为 SocketCAN 网络接口。

## 单电机测试程序

`single_motor_demo` 用于快速测试一个 Robstride 电机。它会执行以下流程：

1. 初始化指定 CAN 接口。
2. 设置为运控模式。
3. 使能候选电机并发送 `Kp=0`、`Kd=4`、`torque=0` 的软状态指令。
4. 等待有效反馈。
5. 未收到反馈的 ID 会立即失能。
6. 持续读取并打印已发现电机的角度和温度。
7. 按下 `Ctrl-C` 后失能程序中使能过的电机，并关闭 CAN。

运行方式：

```bash
ros2 run robstride_can single_motor_demo <can_interface> <motor_id> [RS00|EL05|RS05]
```

示例：

```bash
ros2 run robstride_can single_motor_demo can0 0x06
```

如果不指定电机类型，程序会按电机 ID 选择默认类型：

- `1` 到 `3` 默认使用 `RS00`
- `4` 到 `15` 默认使用 `EL05`

也可以显式指定：

```bash
ros2 run robstride_can single_motor_demo can0 0x06 EL05
```

测试动作默认参数：

- 测试中心：启动后收到的当前反馈位置
- 振幅：`0.15 rad`
- 频率：`0.2 Hz`
- 持续时间：`12 s`
- 控制周期：`10 ms`
- 控制增益：`Kp=20`、`Kd=1`

注意：运行前请确保电机已固定或处于安全测试环境，CAN 接口已启动，急停或断电手段可用。

## 电机发现程序

`discover_motors` 用于扫描 CAN 总线上的 Robstride 电机，并打印发现电机的角度和温度。扫描过程中会让候选电机进入运控模式，并发送 `Kp=0`、`Kd=4`、`torque=0` 的指令，让电机处于“软”状态，不跟踪位置。

注意：Robstride 手册中没有明确可直接读取电机型号的寄存器，因此本程序不识别或显示电机型号；电机型号需要在后续业务配置中手动设置。

运行方式：

```bash
ros2 run robstride_can discover_motors [can_interface] [motor_id0 motor_id1 ...]
```

参数规则：

- 不传参数时，默认使用 `can0`，并扫描 `0~255`。
- 只传 CAN 接口时，扫描该接口上的 `0~255`。
- 第一个参数为空字符串时，也会默认使用 `can0`。
- 从第二个参数开始可以指定一个或多个电机 ID，例如 `0x06 7 8`。
- 最多发现 256 个电机，达到 256 个后会停止继续扫描。
- 发现完成后不会自动退出，会持续读取已发现电机的反馈。
- 退出方式是按 `Ctrl-C`，程序退出前会关闭已使能过的电机。

示例：

```bash
ros2 run robstride_can discover_motors
ros2 run robstride_can discover_motors can0
ros2 run robstride_can discover_motors can0 0x06 7 8
ros2 run robstride_can discover_motors "" 0x06
```

发现电机后会打印类似信息：

```text
发现电机: id=0x06, angle=0.123 rad, temp=31.2 C, mode=2, fault=0x0
```

## 电机零位标定程序

`set_zero_pos` 用于把指定电机的当前位置写入为电机机械零位。该程序只初始化 CAN 驱动并发送零位设置指令，不会使能电机或执行运动。

运行方式：

```bash
ros2 run robstride_can set_zero_pos <can_interface> <motor_id>
```

示例：

```bash
ros2 run robstride_can set_zero_pos can0 0x01
ros2 run robstride_can set_zero_pos can0 1
```

参数规则：

- `can_interface` 是 SocketCAN 接口名，例如 `can0`。
- `motor_id` 可以使用十六进制，例如 `0x01`，也可以使用十进制，例如 `1`。

注意：该操作会改变电机内部零位。运行前请确认关节已经移动到需要作为零点的位置，且 motor ID 填写正确。建议一次只标定一个电机，并确保机械臂处于安全状态。

## 在其他 ROS 2 包中使用

在 `package.xml` 中添加依赖：

```xml
<depend>robstride_can</depend>
```

在 `CMakeLists.txt` 中查找并链接该库：

```cmake
find_package(robstride_can REQUIRED)

ament_target_dependencies(your_target
  robstride_can
)
```

代码中包含头文件：

```cpp
#include "robstride_can/robstride_can_driver.hpp"
```

## 最小示例

```cpp
#include "robstride_can/robstride_can_driver.hpp"

#include <chrono>
#include <thread>

int main()
{
  using robstride_can::MotorType;
  using robstride_can::RobstrideCanDriver;
  using robstride_can::RunMode;

  RobstrideCanDriver driver("can0");
  if (!driver.init()) {
    return 1;
  }

  const uint8_t motor_id = 1;
  driver.setMotorType(motor_id, MotorType::RS00);
  driver.startReceiveThread();

  driver.disableMotor(motor_id, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  driver.setRunMode(motor_id, RunMode::MOTION_CONTROL);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  driver.enableMotor(motor_id);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  driver.sendMotionControl(
    motor_id,
    MotorType::RS00,
    0.0,   // 目标位置，单位 rad
    0.0,   // 目标速度，单位 rad/s
    20.0,  // Kp
    1.0,   // Kd
    0.0);  // 前馈力矩，单位 Nm

  auto feedback = driver.getMotorFeedback(motor_id);
  if (feedback.is_valid) {
    // 可读取 feedback.position、feedback.velocity、feedback.torque、feedback.temperature 等数据
  }

  driver.disableMotor(motor_id);
  driver.stopReceiveThread();
  driver.close();
  return 0;
}
```

## 主要 API

主要类型：

- `RobstrideCanDriver`：Robstride CAN 驱动类。
- `MotorType`：电机类型，包含 `RS00`、`EL05`、`RS05`。
- `RunMode`：运行模式，包含 `MOTION_CONTROL`、`POSITION_PP`、`VELOCITY`、`CURRENT`、`POSITION_CSP`。
- `MotorFeedback`：单个电机的反馈缓存。
- `MotorParams`：电机类型对应的协议映射范围。

常用方法：

- `init()`：打开并绑定 SocketCAN 接口。
- `close()`：关闭 CAN socket。
- `isConnected()`：检查 CAN socket 是否已打开。
- `setMotorType(motor_id, type)`：设置指定电机 ID 的电机类型，用于反馈解码。
- `startReceiveThread()` / `stopReceiveThread()`：启动或停止后台反馈接收线程。
- `enableMotor(motor_id)`：发送电机使能命令。
- `disableMotor(motor_id, clear_fault)`：停止电机，可选择清除故障。
- `setZeroPosition(motor_id)`：设置电机机械零位。
- `setRunMode(motor_id, mode)`：写入运行模式参数。
- `setPositionCSP(motor_id, position)`：写入 CSP 位置参考值。
- `setVelocityLimit(motor_id, velocity_limit)`：写入速度上限。
- `sendMotionControl(...)`：发送 Type 1 运控模式控制指令。
- `getMotorFeedback(motor_id)`：获取指定电机最新的反馈缓存。
- `getSendRetryCount()` / `getSendFailCount()`：获取 CAN 发送重试和失败统计。

## 注意事项

- `enableMotor()` 不会自动设置运行模式；如果目标模式有要求，需要先调用 `setRunMode()`。
- 反馈解码依赖 `setMotorType()` 配置的电机类型。
- 运控模式下，速度按照协议中的 `[-50, 50] rad/s` 范围编码。
- 接收 socket 当前过滤为 Type 2 电机反馈帧。
- 本库不负责创建或配置 Linux CAN 接口，使用前需要先将接口启动。

## 许可证

Apache-2.0
