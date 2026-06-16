# easyarm_utils

EasyArm 数据和调试工具。

## plot_ee_trajectory

绘制 `easyarm_record` 录制 JSON 中的末端 3D 轨迹。

```bash
ros2 run easyarm_utils plot_ee_trajectory data/20260525/19-50-13.json
```

## keyboard_teleop

键盘控制真实机械臂末端。启动确认后会读取一次当前 `Link6` 位姿作为内部 `target_pose`，后续按键只在该目标位姿上累加增量，不再用每次按键时的实时末端位姿作为增量基准。随后工具会发送当前点 hold trajectory，再将 hardware 切换到 `POSITION` 模式。按键移动会向 `arm_controller/joint_trajectory` 流式发布短前瞻轨迹，减少每步等待 action 完成带来的卡顿。所有位移和姿态增量都按 `Link6` 局部坐标系解释。

```bash
ros2 run easyarm_utils keyboard_teleop
```

参数示例：

```bash
ros2 run easyarm_utils keyboard_teleop --ros-args -p linear_step:=0.003 -p angular_step_deg:=1.0 -p stream_duration:=0.10
```

按键说明：

| 按键 | 行为 |
|------|------|
| I/K | 沿 `Link6` 局部 Z 前进/后退，前进为 `-Z`，后退为 `+Z` |
| J/L | 沿 `Link6` 局部 Y 左/右，左为 `-Y`，右为 `+Y` |
| 空格/C | 沿 `Link6` 局部 X 上升/下降 |
| W/S | pitch 正/负方向 |
| A/D | yaw 正/负方向 |
| Q/E | roll 正/负方向 |
| Esc | hold 当前点并退出 |

该工具会控制真实机械臂，运行前必须确认硬件和工作空间安全。
