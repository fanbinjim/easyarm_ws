# 钛虎 Pro-2 系列关节模组 CAN 通讯协议

## 1. 通讯概述

- **通讯类型**：经典 CAN（非 CAN FD）
- **默认波特率**：1 Mbps
- **数据帧类型**：标准帧（11-bit CAN ID）
- **CAN ID 分配**：每个电机模组有唯一 CAN ID，出厂默认 `0x01`
- **数据长度**：`1/5/6/7/8` 字节，不同长度对应不同指令类型
- **字节序**：低位在前，高位在后（Little-Endian）
- **典型周期**：7 个电机组网约 300 Hz
- **FDCAN 支持**：文档仅提及 FDCAN 支持更长数据帧（最多 64 字节），但未明确说明电机支持 FDCAN。当前协议基于经典 CAN，数据长度限制在 1-8 字节

## 2. 组网要求

- 每个电机模组必须配置唯一 CAN ID
- CAN 总线两端需要 120Ω 终端电阻
- 1 Mbps 波特率下建议总线长度 ≤ 40m
- 电机支持 CAN 独立使用或 CAN + EtherCAT 共存

## 3. CAN 与 EtherCAT 的关系

- 电机同时支持 EtherCAT 和 CAN
- **EtherCAT 控制优先级高于 CAN**
- 当 EtherCAT 和 CAN 同时连接时，**CAN 只能监控状态，不能控制电机**
- 断开 EtherCAT 后，CAN 可正常控制

## 4. 协议总体格式

```
CAN ID = 电机 CAN ID
data[0] = 指令码
data[1..N] = 参数（小端序）
```

根据 `data` 长度区分指令类型：

| 数据长度 | 指令类型 |
|----------|----------|
| 1 字节 | 状态查询/控制指令 |
| 5 字节 | 控制指令 + int32 参数 |
| 6 字节 | 参数读写指令 |
| 7 字节 | 前馈位置模式指令 |
| 8 字节 | 轮廓位置模式/综合反馈指令 |

## 5. 一字节指令

| 指令码 | 功能 | 回复 |
|--------|------|------|
| `0x02` | 电机去使能 | 无 |
| `0x03` | 获取运行模式 | 5 字节 |
| `0x04` | 获取反馈电流 | 5 字节 |
| `0x06` | 获取电机速度 | 5 字节 |
| `0x08` | 获取电机反馈位置 | 5 字节 |
| `0x0A` | 获取电机报错状态 | 5 字节 |
| `0x0B` | 清除电机报错 | 无 |
| `0x0E` | 保存参数到 flash | 无 |

## 6. 五字节控制指令

格式：`data[0]` = 指令码，`data[1..4]` = int32 参数（小端序）

### 6.1 电流/速度/位置控制

| 指令码 | 功能 | 参数单位 |
|--------|------|----------|
| `0x1C` | 设置目标电流 | mA |
| `0x1D` | 设置目标速度 | 0.01 Hz（即 0.01 rps） |
| `0x1E` | 设置目标位置 | encoder cnt |
| `0x20` | 设置最大电流 | mA |
| `0x21` | 设置最小电流 | mA |
| `0x22` | 设置加速度 | — |
| `0x23` | 设置减速度 | — |
| `0x24` | 设置最大速度 | 0.01 Hz |
| `0x25` | 设置最小速度 | 0.01 Hz |
| `0x26` | 设置最大位置 | encoder cnt |
| `0x27` | 设置最小位置 | encoder cnt |

### 6.2 配置指令

| 指令码 | 功能 | 参数说明 |
|--------|------|----------|
| `0x2E` | 设置电机 CAN ID | 1-255 |
| `0x3F` | 设置 CAN 波特率 | 见波特率表 |

### 6.3 控制并获取反馈

| 指令码 | 功能 | 回复长度 |
|--------|------|----------|
| `0x42` | 设置目标电流并返回反馈 | 8 字节 |
| `0x43` | 设置目标速度并返回反馈 | 8 字节 |
| `0x44` | 设置目标位置并返回反馈 | 8 字节 |

### 6.4 其他控制

| 指令码 | 功能 | 参数说明 |
|--------|------|----------|
| `0x53` | 设置位置偏置 | int32 |
| `0x55` | 设置位置限制功能 | 0：使能，1：失效 |
| `0x5D` | 设置步进电机细分 | — |
| `0x64` | 设置堵转保护阈值 | 100~10000 |
| `0x65` | 设置堵转保护速度 | 0~100 |
| `0x70` | 设置编码器工作模式 | 0：单编，1：双编 |
| `0x71` | 设置 8 字节指令控制模式 | 0：PT 模式，1：轮廓位置模式 |
| `0x80` | 电机正向校准 | — |
| `0x81` | 电机反向校准 | — |
| `0x82` | 电机额定电流校准 | — |
| `0x83` | 电机 PWM 校准 | — |
| `0x84` | 电机 PID 校准 | — |
| `0x85` | 编码器校准 | — |
| `0x86` | 电机退磁 | — |

## 7. 六字节参数读写指令

格式：

```
data[0]     = 指令码
data[1]     = 读写标志
data[2..5]  = 参数（int32，小端序）
```

读写标志：

| 标志值 | 含义 |
|--------|------|
| `0x20` | 写参数 |
| `0x40` | 读参数 |

指令列表：

| 指令码 | 功能 |
|--------|------|
| `0x41` | 设置/读取减速比 |
| `0x42` | 设置/读取 PT 模式 KP |
| `0x43` | 设置/读取 PT 模式 KD |
| `0x44` | 设置/读取 PT 模式 KT |
| `0x46` | 设置/读取 PT 模式 Tmin |
| `0x47` | 设置/读取 PT 模式 Tmax |
| `0x48` | 设置/读取 PT 模式 Imin |
| `0x49` | 设置/读取 PT 模式 Imax |
| `0x96` | 设置/读取温度传感器类型 |
| `0x9D` | 设置/读取位置控制方式 |

## 8. 七字节前馈位置模式指令

### 8.1 前馈电流位置模式 (`0x10`)

```
data[0]     = 指令码 0x10
data[1..4]  = 目标位置（int32，小端序，单位 cnt）
data[5..6]  = 电流前馈（int16，小端序，单位 0.1A）
```

### 8.2 前馈速度位置模式 (`0x11`)

```
data[0]     = 指令码 0x11
data[1..4]  = 目标位置（int32，小端序，单位 cnt）
data[5..6]  = 速度前馈（int16，小端序，单位 0.01Hz）
```

### 8.3 限速度位置模式 (`0x58`)

```
data[0]     = 指令码 0x58
data[1..4]  = 目标位置（int32，小端序，单位 cnt）
data[5..6]  = 最大速度（int16，小端序，单位 0.01Hz）
```

## 9. 八字节轮廓位置模式指令 (`0x12`)

### 发送格式

```
data[0..3]  = 目标位置（int32，小端序，单位 cnt）
data[4..5]  = 轮廓速度（int16，小端序，单位 0.01Hz）
data[6..7]  = 控制字（uint16，小端序）
```

### 返回格式（8 字节）

```
data[0..3]  = 反馈位置（int32，小端序，单位 cnt）
data[4..5]  = 反馈速度（int16，小端序，单位 0.01Hz）
data[6..7]  = 状态字（uint16，小端序）
```

### 控制字

| 控制字 | 含义 |
|--------|------|
| `0x001F` | 顺序模式运行 |
| `0x003F` | 立即模式运行 |
| bit 8 | halt（急停） |
| bit 6 | 相对位置模式 |

### 状态字

| bit | 含义 |
|-----|------|
| bit 12 | 已接收新目标并更新轨迹 |
| bit 10 | 上一个指令轨迹已完成 |

## 10. 电机工作模式

### 10.1 PT 模式（力位混合模式）

PT 模式是钛虎电机的特色控制模式，通过位置（P）和力矩（T）混合控制电机运动。

**控制参数**：

| 参数 | 说明 |
|------|------|
| KP | 位置比例增益 |
| KD | 速度比例增益 |
| KT | 力矩比例增益 |
| Tmin | 最小力矩限制 |
| Tmax | 最大力矩限制 |
| Imin | 最小电流限制 |
| Imax | 最大电流限制 |

**切换到 PT 模式**：

```
设置 0x71 指令，参数 = 0（PT 模式）
```

### 10.2 轮廓位置模式（CSP/PP）

轮廓位置模式支持梯形速度规划，电机自动加减速。

**控制参数**：
- 目标位置（encoder cnt）
- 轮廓速度（0.01Hz）
- 加速度/减速度（通过 0x22/0x23 指令设置）

**切换到轮廓位置模式**：

```
设置 0x71 指令，参数 = 1（轮廓位置模式）
```

### 10.3 电流模式

直接控制电机输出电流（力矩）。

**使用指令**：

```
0x1C：设置目标电流（mA）
0x42：设置目标电流并获取反馈
```

### 10.4 速度模式

直接控制电机转速。

**使用指令**：

```
0x1D：设置目标速度（0.01Hz）
0x43：设置目标速度并获取反馈
```

### 10.5 位置模式

直接设置目标位置，电机以最大速度运动到目标位置。

**使用指令**：

```
0x1E：设置目标位置（cnt）
0x44：设置目标位置并获取反馈
```

### 10.6 前馈位置模式

在位置控制的基础上叠加前馈信号，提高跟踪性能。

- **前馈电流位置模式** (`0x10`)：位置环 + 电流前馈
- **前馈速度位置模式** (`0x11`)：位置环 + 速度前馈
- **限速度位置模式** (`0x58`)：位置环 + 速度限制

### 10.7 工作模式查询

使用 `0x03` 指令获取当前运行模式，返回 5 字节。

## 11. 八字节综合反馈

使用 `0x42/0x43/0x44` 指令或查询 `0x41` 指令时，返回 8 字节反馈数据：

```
data[0..1] = 电机速度（int16，小端序，单位 0.01Hz）
data[2..3] = 电机电流（int16，小端序，单位 mA）
data[4..7] = 电机位置（int32，小端序，单位 cnt）
```

## 12. 错误码

使用 `0x0A` 指令获取错误状态，返回 5 字节，`data[1..4]` 为错误码（int32，小端序）。

| bit | 说明 |
|-----|------|
| bit 0 | 过压 |
| bit 1 | 欠压 |
| bit 2 | 过流 |
| bit 3 | 软件错误 |
| bit 4 | 位置超限 |
| bit 5 | 速度超限 |
| bit 6 | 电流超限 |
| bit 7 | 堵转 |
| bit 8 | 温度异常 |
| bit 9 | 编码器故障 |
| bit 10 | 参数错误 |
| bit 11 | 通讯故障 |

使用 `0x0B` 指令清除错误。

## 13. 单位换算

### 13.1 位置换算

**单编码器模式**（`0x70` 指令设置为 0）：

```
输出端角度(°) = cnt / 65536 / 减速比 × 360
```

**双编码器模式**（`0x70` 指令设置为 1）：

```
输出端角度(°) = cnt / 262144 × 360
```

### 13.2 速度换算

```
电机端 RPM = value × 0.6    （value 单位为 0.01Hz）
输出端 RPM = 电机端 RPM / 减速比
```

### 13.3 电流换算

- 大部分指令电流单位为 mA
- 前馈电流（`0x10` 指令）单位为 0.1A

### 13.4 减速比

减速比可通过 `0x41` 指令查询，默认出厂值根据型号不同而不同。

## 14. 驱动实现建议

### 14.1 指令封装

建议将协议指令封装为以下层次：

```
底层：sendCommand(motor_id, cmd_code, data, len)
中层：writeInt32Param(motor_id, cmd_code, value)
      readInt32Param(motor_id, cmd_code)
高层：setTargetPosition(motor_id, position_cnt)
      getMotorFeedback(motor_id)
```

### 14.2 单位转换

建议在驱动层提供单位转换工具函数：

```cpp
// 角度(cnt) <-> 弧度(rad)
double cntToRad(int32_t cnt, double gear_ratio, bool dual_encoder);
int32_t radToCnt(double rad, double gear_ratio, bool dual_encoder);

// 速度(0.01Hz) <-> rad/s
double hzToRadPerSec(int16_t hz);
int16_t radPerSecToHz(double rad_s);
```

### 14.3 反馈解析

对于 `0x42/0x43/0x44` 返回的 8 字节反馈：

```cpp
struct MotorFeedback8 {
    int16_t velocity;   // data[0..1]，单位 0.01Hz
    int16_t current;    // data[2..3]，单位 mA
    int32_t position;   // data[4..7]，单位 cnt
};
```

### 14.4 命名规范

建议命令码常量命名：

```cpp
constexpr uint8_t CMD_DISABLE          = 0x02;
constexpr uint8_t CMD_GET_MODE         = 0x03;
constexpr uint8_t CMD_GET_CURRENT      = 0x04;
constexpr uint8_t CMD_GET_VELOCITY     = 0x06;
constexpr uint8_t CMD_GET_POSITION     = 0x08;
constexpr uint8_t CMD_GET_ERROR        = 0x0A;
constexpr uint8_t CMD_CLEAR_ERROR      = 0x0B;
constexpr uint8_t CMD_SAVE_FLASH       = 0x0E;
constexpr uint8_t CMD_SET_CURRENT      = 0x1C;
constexpr uint8_t CMD_SET_VELOCITY     = 0x1D;
constexpr uint8_t CMD_SET_POSITION     = 0x1E;
constexpr uint8_t CMD_SET_MAX_CURRENT  = 0x20;
constexpr uint8_t CMD_SET_MIN_CURRENT  = 0x21;
constexpr uint8_t CMD_SET_ACCEL        = 0x22;
constexpr uint8_t CMD_SET_DECEL        = 0x23;
constexpr uint8_t CMD_SET_MAX_VELOCITY = 0x24;
constexpr uint8_t CMD_SET_MIN_VELOCITY = 0x25;
constexpr uint8_t CMD_SET_MAX_POSITION = 0x26;
constexpr uint8_t CMD_SET_MIN_POSITION = 0x27;
constexpr uint8_t CMD_SET_CAN_ID       = 0x2E;
constexpr uint8_t CMD_SET_BAUDRATE     = 0x3F;
constexpr uint8_t CMD_SET_CURRENT_FB   = 0x42;
constexpr uint8_t CMD_SET_VELOCITY_FB  = 0x43;
constexpr uint8_t CMD_SET_POSITION_FB  = 0x44;
constexpr uint8_t CMD_PROFILE_POSITION = 0x12;
```
