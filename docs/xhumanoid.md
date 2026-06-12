# XHumanoid 电机使用说明

本文记录 `easyarm_can` 中 XHumanoid/HRA 关节模组的当前使用方法。协议依据主要来自
`src/easyarm_can/ref/xhumanoid/CAN协议说明E_V1.2.pdf`、厂家示例代码，以及本仓库现场
测试记录。

## 1. 协议和硬件行为

- 通信类型：CAN 2.0B 标准帧。
- 波特率：`1Mbps`。
- DLC：固定 `8` 字节。
- CAN ID：普通控制和反馈使用关节 ID，例如 `0x001`。
- 控制方式：XHumanoid 该协议下没有单独的 enable/disable 功能，周期发送控制报文即运动，停止发送即停止。
- 推荐发送周期：`5ms`。
- 温度报文：关节约 `1s` 自动上报一次，示例 `A0 0A 00 7B 00 79 7F B3` 表示绕组温度 `36.5 degC`、MOS 温度 `35.5 degC`。
- 位置模式公共接口使用 `PositionCommand`，上层单位为 `rad`、`rad/s`、`A`。
- 速度模式公共接口使用 `VelocityCommand`，上层单位为 `rad/s`、`A`。

## 2. SocketCAN 配置

真实硬件测试前先配置 `can0`：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

查看总线报文：

```bash
candump can0
```

## 3. 手工 cangen 测试

速度模式示例，ID 为 `0x001`：

```bash
cangen can0 -g 5 -I 001 -L 8 -D 413F800000000AFF
```

力位混合模式示例，`kp=50`、`kd=10`、`q=0`、`dq=0`、`tau=0`：

```bash
cangen can0 -g 5 -I 001 -L 8 -D 00CC117FFF7FF7FF
```

力位混合模式示例，`kp=50`、`kd=10`、`q=3.14 rad`、`dq=0`、`tau=0`：

```bash
cangen can0 -g 5 -I 001 -L 8 -D 00CC11BFFF7FF7FF
```

位置模式示例，`pos=1.0 rad`、`velocity=0.5 rad/s`、`current_limit=2.0 A`：

```bash
ros2 run easyarm_can test_xhumanoid --mode position \
  --pos 1.0 --vel 0.5 --current-limit 2.0 --dryrun
```

速度模式示例，`velocity=0.1 rad/s`、`current_limit=1.0 A`：

```bash
ros2 run easyarm_can test_xhumanoid --mode velocity \
  --vel 0.1 --current-limit 1.0 --dryrun
```

## 4. ros2 run 测试程序

构建并加载环境：

```bash
colcon build --packages-select easyarm_can
source install/setup.bash
```

列出内置 XHumanoid 型号：

```bash
ros2 run easyarm_can test_xhumanoid --list-models
```

打印 payload 但不发送 CAN 帧：

```bash
ros2 run easyarm_can test_xhumanoid --kp 10 --kd 5 --pos 3.14 --vel 0 --torque 0 --dryrun
```

位置模式 dry-run：

```bash
ros2 run easyarm_can test_xhumanoid --mode position \
  --pos 1.0 --vel 0.5 --current-limit 2.0 --dryrun
```

确认机械限位和方向安全后，不加 `--dryrun` 会周期发送：

```bash
ros2 run easyarm_can test_xhumanoid \
  --model xhumanoid_60h_100 \
  --kp 10 --kd 5 --pos 3.14 --vel 0 --torque 0 \
  --cycles 1000 --period-ms 5
```

常用参数：

| 参数 | 说明 |
| --- | --- |
| `--can` | CAN 接口，默认 `can0` |
| `--id` | 电机 CAN ID，默认 `1` |
| `--model` | 电机型号，默认 `xhumanoid_60h_100` |
| `--mode` | 控制模式，`hybrid`、`position` 或 `velocity`，默认 `hybrid` |
| `--pos` | 目标位置，单位 `rad` |
| `--vel` | 速度参数，单位 `rad/s`；位置模式中按厂商协议可解释为速度目标或速度上限 |
| `--current-limit` | 位置/速度模式电流阈值，单位 `A` |
| `--kp` | 力位混合模式 KP |
| `--kd` | 力位混合模式 KD |
| `--torque` | 前馈扭矩，单位 `Nm` |
| `--cycles` | 发送周期数 |
| `--period-ms` | 发送周期，默认 `5ms` |
| `--dryrun` | 只打印 payload，不打开 CAN，也不发送 CAN 帧 |

## 5. 力位混合模式编码范围

当前驱动按 `CAN协议说明E_V1.2.pdf` 的范围编码：

| 字段 | 位宽 | 范围 | 单位 |
| --- | ---: | --- | --- |
| `kp` | 12 | `0` ~ `2000` | 协议增益 |
| `kd` | 9 | `0` ~ `300` | 协议增益 |
| `q` | 16 | `-6.28` ~ `6.28` | `rad` |
| `dq` | 12 | `-21` ~ `21` | `rad/s` |
| `tau` | 12 | `-300` ~ `300` | `Nm` |

量化函数使用厂家示例中的截断方式：

```text
raw = int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))
```

## 6. 位置模式编码

公共接口：

```cpp
easyarm_can::PositionCommand command;
command.position_rad = 1.0;
command.velocity_rad_s = 0.5;
command.current_limit_a = 2.0;
driver.sendPositionControl(motor_id, command);
```

CAN 2.0B 位置模式按厂家 `set_motor_position` 打包；公共 API 使用 `rad`，驱动写入总线前转换为 `deg`：

| 字段 | 编码 |
| --- | --- |
| 电机模式 | `Byte0[5:7] = 0x01`，即高 3 位为 `0x20` |
| 目标位置 | `radToDeg(position_rad)` 的 float32 大端位流 |
| 速度参数 | `round(radPerSecToRpm(velocity_rad_s) * 10)` |
| 电流阈值 | `round(current_limit_a * 10)` |
| 报文返回 | 驱动固定写 `0x01`，只请求报文 1 返回 |

CAN FD 位置模式发送 DLC `14`；位置字段同样使用 `deg`：

| Byte | 含义 |
| --- | --- |
| 1 | `0x12` |
| 2~5 | `radToDeg(position_rad)` float32 大端 |
| 6~9 | `velocity_rad_s` float32 大端 |
| 10~13 | `current_limit_a` float32 大端 |
| 14 | 自增计数 |

## 7. 速度模式编码

公共接口：

```cpp
easyarm_can::VelocityCommand command;
command.velocity_rad_s = 0.1;
command.current_limit_a = 1.0;
driver.sendVelocityControl(motor_id, command);
```

CAN 2.0B 速度模式按厂家 `set_motor_speed` 打包；公共 API 使用 `rad/s`，驱动写入总线前转换为输出端 `rpm`。报文返回字段固定为 `0x01`，即 `Byte0=0x41`：

| 字段 | 编码 |
| --- | --- |
| 电机模式 | `Byte0[5:7] = 0x02`，即高 3 位为 `0x40` |
| 报文返回 | 驱动固定写 `0x01`，只请求报文 1 返回 |
| 目标速度 | `radPerSecToRpm(velocity_rad_s)` float32 大端 |
| 电流阈值 | `round(current_limit_a * 10)` 的 uint16 大端，raw 按协议夹紧到 `0~3000` |
| 固定字节 | `Byte7 = 0xFF` |

`velocity_rad_s=1.0`、`current_limit_a=1.0` 的 CAN 2.0B payload 为：

```text
414118C9EB000AFF
```

CAN FD 速度模式发送 DLC `10`：

| Byte | 含义 |
| --- | --- |
| 1 | `0x13` |
| 2~5 | `velocity_rad_s` float32 大端 |
| 6~9 | `current_limit_a` float32 大端 |
| 10 | 自增计数 |

## 8. 反馈解析

驱动解析两类反馈：

- 报文 1：位置、速度、电流。
- 自动温度报文：绕组温度、MOS 温度。

公共反馈结构 `MotorFeedback` 当前只有一个温度字段，所以温度报文中取绕组和 MOS 的较高值写入 `temperature_deg_c`。

XHumanoid 报文反馈的是电流，驱动用型号表中的 `torque_constant_nm_per_a` 换算为扭矩：

```text
feedback.torque_nm = current_a * torque_constant_nm_per_a
```

扭矩系数参考：

```text
src/easyarm_can/ref/xhumanoid/HRA关节模组扭矩系数及电流计算公式.md
```

## 9. 代码位置

- 驱动实现：`src/easyarm_can/src/vendors/xhumanoid/xhumanoid_driver.cpp`
- 测试程序：`src/easyarm_can/example/test_xhumanoid.cpp`
- 型号表：`src/easyarm_can/src/model_registry.cpp`
- 协议资料：`src/easyarm_can/ref/xhumanoid/`

## 10. 安全注意事项

- 不加 `--dryrun` 会让电机真实运动；发送前确认关节机械限位、方向、负载和急停条件。
- XHumanoid 没有单独 disable 指令，测试程序结束后靠停止发送控制帧使电机停止。
- 首次测试建议使用较小 `kp/kd`，并从当前位置附近的小幅 `pos` 开始。
- 首次测试位置模式建议使用较小 `--vel` 和 `--current-limit`，并从当前位置附近的小幅 `--pos` 开始。
- 首次测试速度模式建议使用较小 `--vel` 和 `--current-limit`，确认方向后再逐步提高速度。
- 如果反馈扭矩一直为 `0`，先确认 `--model` 是否选择了带有效 `torque_constant_nm_per_a` 的型号。
