# 巨蟹智能 MIT 模式摘要

资料来源：TN0001 MIT 模式 CANopen 使用教程，Rev. 1.0，2026-05-11。

## 控制量

MIT 命令包含 5 个关节侧物理量：

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `q_des` | 期望关节位置 | `rad` |
| `dq_des` | 期望关节速度 | `rad/s` |
| `Kp` | 位置刚度 | `Nm/rad` |
| `Kd` | 速度阻尼 | `Nm*s/rad` |
| `tau_ff` | 力矩前馈 | `Nm` |

控制器含义：

```text
tau_ref = Kp * (q_des - q_meas) + Kd * (dq_des - dq_meas) + tau_ff
```

协议中的量是关节侧单位。固件内部负责关节力矩到电流的换算，并会进行位置、速度、增益、力矩、电流、力矩变化率和超时保护裁剪。

## 控制入口

| 入口 | CAN ID | DLC | 说明 |
| --- | --- | --- | --- |
| PDO4/MIT 固定通道 | `0x500 + Dev_ID` | 8 | 全量 CANopen；只发送 MIT payload；需已进入 Operation enabled + MIT 模式 |
| MIT 单轴快控 | `0x110 + Dev_ID` | 9 | Byte0 控制位，Byte1..8 为 MIT payload |
| MIT 多轴快控 | `0x210` | 64 | 最多 6 个 9B MIT 子帧 |

`0x500 + Dev_ID` 是固件固定 MIT 通道，借用 RPDO4 COB-ID，但不按对象字典 `1603h` 映射解析，也不支持通过 SDO 修改 payload 映射。

## 全量 CANopen MIT 流程

以 `Dev_ID = 1` 为例：

| 步骤 | CAN ID | DLC | Data | 说明 |
| --- | --- | --- | --- | --- |
| 1 | `0x000` | 2 | `01 01` | NMT 启动节点 1 |
| 2 | `0x601` | 8 | `2F 60 60 00 C0 00 00 00` | 写 `6060h = -64`，进入 MIT 模式 |
| 3 | `0x601` | 8 | `2B 40 60 00 06 00 00 00` | `6040h = 0x0006` |
| 4 | `0x601` | 8 | `2B 40 60 00 07 00 00 00` | `6040h = 0x0007` |
| 5 | `0x601` | 8 | `2B 40 60 00 0F 00 00 00` | `6040h = 0x000F` |
| 6 | `0x501` | 8 | `80 00 80 03 E8 14 18 00` | 周期下发 MIT payload |

推荐发送周期 `1 ms..10 ms`。周期过长会触发 MIT 命令超时保护。默认通常约 `50 ms` 后进入安全阻尼，继续约 `500 ms` 后请求 quick stop；具体阈值可能随设备参数变化。

## 8B MIT payload

固定 8 字节，大端位打包：

| 字段 | 位宽 | 位置 | 物理量 | 映射范围 |
| --- | --- | --- | --- | --- |
| `q_des` | 16 bit | Byte0..1 | 期望关节位置 | `-P_MAX..+P_MAX rad` |
| `dq_des` | 12 bit | Byte2 + Byte3 高 4 bit | 期望关节速度 | `-V_MAX..+V_MAX rad/s` |
| `Kp` | 12 bit | Byte3 低 4 bit + Byte4 | 位置刚度 | `0..4095 Nm/rad` |
| `Kd` | 12 bit | Byte5 + Byte6 高 4 bit | 速度阻尼 | `0..255 Nm*s/rad` |
| `tau_ff` | 12 bit | Byte6 低 4 bit + Byte7 | 力矩前馈 | `-T_MAX..+T_MAX Nm` |

范围含义：

- `P_MAX`：当前生效的位置映射半量程。若启用关节行程限位或软件位置限位，取正负限位绝对值较大者；否则回退 `pi rad`。
- `V_MAX`：当前 MIT payload 解包使用的关节侧速度半量程。通常由电机最大转速除以减速比后换算为 `rad/s`。
- `T_MAX`：设备当前可用关节力矩限制，受力矩参数、电流限制和电机配置影响。
- `Kp` 满量程固定为 `4095 Nm/rad`。
- `Kd` 满量程固定为 `255 Nm*s/rad`。

位打包：

```text
Byte0 = q_des[15:8]
Byte1 = q_des[7:0]
Byte2 = dq_des[11:4]
Byte3 = dq_des[3:0] << 4 | Kp[11:8]
Byte4 = Kp[7:0]
Byte5 = Kd[11:4]
Byte6 = Kd[3:0] << 4 | tau_ff[11:8]
Byte7 = tau_ff[7:0]
```

不要把全 0 payload 当作有效 MIT 命令。停止或失能应通过 CANopen `6040h` 控制字完成。

## 组包换算

浮点量转无符号整数：

```text
raw = round((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))
```

发送前需要限幅到 `[x_min, x_max]`。

示例参数：

| 参数 | 值 |
| --- | --- |
| `P_MAX` | `pi rad` |
| `MOTOR_MAX_SPEED_RPM` | `3000 rpm` |
| `GEAR_RATIO` | `9` |
| `V_MAX` | `3000 / 9 rpm = 34.9066 rad/s` |
| `T_MAX` | `10 Nm` |
| `q_des` | `0 rad` |
| `dq_des` | `0 rad/s` |
| `Kp` | `1000 Nm/rad` |
| `Kd` | `20 Nm*s/rad` |
| `tau_ff` | `0 Nm` |

组包结果：

```text
80 00 80 03 E8 14 18 00
```

完整 PDO4/MIT 帧，`Dev_ID = 1`：

```text
0x501 DLC 8: 80 00 80 03 E8 14 18 00
```

## 单轴 MIT 快控

CAN ID：`0x110 + Dev_ID`，DLC：`9`。

Byte0 控制位：

| 位 | 含义 |
| --- | --- |
| Bit7 | 使能，`1` 上使能 |
| Bit6 | 抱闸，`1` 抱闸释放 |
| Bit5 | 清错，`1` 触发清错 |
| Bit4..1 | 控制模式，MIT 固定为 `0x06` |
| Bit0 | 保留 |

MIT 使能并释放抱闸时：

```text
0x80 | 0x40 | (0x06 << 1) = 0xCC
```

沿用上文 payload，完整单轴快控帧：

```text
0x111 DLC 9: CC 80 00 80 03 E8 14 18 00
```

当 Byte0 中 `enable = 1` 时，Byte1..8 的 MIT payload 必须有效，不能为全 0。

## 多轴 MIT 快控

CAN ID：`0x210`，DLC：`64`，最多 6 个 MIT 子帧。

| 字节范围 | 含义 |
| --- | --- |
| Byte0..8 | 槽位 1 的 9B MIT 子帧 |
| Byte9..17 | 槽位 2 的 9B MIT 子帧 |
| Byte18..26 | 槽位 3 的 9B MIT 子帧 |
| Byte27..35 | 槽位 4 的 9B MIT 子帧 |
| Byte36..44 | 槽位 5 的 9B MIT 子帧 |
| Byte45..53 | 槽位 6 的 9B MIT 子帧 |
| Byte54..55 | 保留，填 `00 00` |
| Byte56..61 | 槽位对应的 `Dev_ID` 列表 |
| Byte62..63 | 保留，填 `00 00` |

## 反馈确认

- 全量 CANopen 接入时，可 SDO 读取 `6061h`，返回 `-64` 表示当前显示模式为 MIT。
- 可 SDO 读取 `6041h` 或 TPDO 状态字确认进入 Operation enabled。
- 使用 `0x110/0x210` 快控时，查看 `0x300 + Dev_ID` 反馈；状态机实际进入 MIT 后，Byte10 模式反馈为 `0x06`。

## 调试建议

- 首次调试先发送 `Kp = 0`、`Kd = 0`、`tau_ff = 0` 的非零安全 payload，确认模式、使能和周期通信。
- 纯阻尼测试先保持 `Kp = 0`，逐步增加 `Kd`。
- 位置保持先从低 `Kp` 开始，逐步增加，并监控电流、温度和错误码。
- `tau_ff` 会直接影响输出力矩，空载时可能持续加速，应从小值开始。
- 确认急停、电源限流、机械限位和负载状态后再进入 MIT 控制。
