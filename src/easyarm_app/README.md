# easyarm_app

`easyarm_app` 是新的上层 app/CLI 层。规划式运动调用 `easyarm_motion_server`
暴露的接口；`speedj/speedl` 只发布 MoveIt Servo 的原生速度输入 topic，不直接发底层
trajectory，也不直接访问硬件参数服务。

旧的 `easyarm_move_task` 暂时保留，后续功能稳定后再逐步迁移。

## 命令

```bash
ros2 run easyarm_app easyarm --help
```

常驻 shell：

```bash
ros2 run easyarm_app easyarm_shell
```

shell 支持上下箭头浏览历史命令，历史默认保存到：

```text
easyarm_shell 执行文件同目录下的 .easyarm_shell_history
```

进入 shell 后可直接输入同样的子命令，例如：

```text
easyarm> get-joints
easyarm> get-pose
easyarm> movej 0 0 2.35619 0.7854 -1.5708 0 --plan-only
easyarm> speedj_teleop
easyarm> speedl_teleop
easyarm> exit
```

当前子命令：

```text
movej
movel
set-mode
stop
get-state
get-joints
get-pose
speedj
speedl
speedj_teleop
speedl_teleop
```

## 测试前启动

先启动 `easyarm_a1_bringup`。它会同时启动 MoveIt、ros2_control、controllers 和 `easyarm_motion_server`。

Mock：

```bash
ros2 launch easyarm_a1_bringup bringup.launch.py use_mock_hardware:=true moveit_servo:=true
```

真实硬件：

```bash
ros2 launch easyarm_a1_bringup bringup.launch.py moveit_servo:=true
```

真实硬件启动前需要先配置 `can0`：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## 示例

查询状态：

```bash
ros2 run easyarm_app easyarm get-state
```

查询当前关节状态：

```bash
ros2 run easyarm_app easyarm get-joints
```

查询当前末端位姿：

```bash
ros2 run easyarm_app easyarm get-pose
ros2 run easyarm_app easyarm get-pose --target-frame base_link --source-frame Link6
```

MoveJ 只规划：

```bash
ros2 run easyarm_app easyarm movej 0 0 2.35619 0.7854 -1.5708 0 --plan-only
```

MoveJ 执行：

```bash
ros2 run easyarm_app easyarm movej 0 0 2.35619 0.7854 -1.5708 0 \
  --velocity-scale 0.2 \
  --acceleration-scale 0.2
```

真实硬件低速 MoveJ：

```bash
ros2 run easyarm_app easyarm movej 0 0 2.35619 0.7854 -1.5708 0 \
  --velocity-scale 0.05 \
  --acceleration-scale 0.05
```

MoveL 只规划：

```bash
ros2 run easyarm_app easyarm movel 0.25 0.0 0.25 0.0 0.0 0.0 1.0 --plan-only
```

模式切换：

```bash
ros2 run easyarm_app easyarm set-mode POSITION
ros2 run easyarm_app easyarm set-mode IDLE
ros2 run easyarm_app easyarm set-mode DRAG
```

停止：

```bash
ros2 run easyarm_app easyarm stop
```

SpeedJ 关节速度遥操：

```bash
ros2 run easyarm_app easyarm set-mode POSITION
ros2 run easyarm_app easyarm speedj 0 0.05 0 0 0 0 --duration 1.0 --rate 50
```

SpeedL 末端速度遥操：

```bash
ros2 run easyarm_app easyarm set-mode POSITION
ros2 run easyarm_app easyarm speedl 0.01 0 0 0 0 0 --duration 1.0 --rate 50
```

## 键盘 SpeedJ 模式

`easyarm_shell` 中输入 `speedj_teleop` 会进入键盘关节速度控制模式：

```text
easyarm> speedj_teleop
```

按键映射：

```text
1 2 3 4 5 6  -> Joint1..Joint6 正方向速度
q w e r t y  -> Joint1..Joint6 负方向速度
Esc          -> 退出并发送零速度
```

按住按键时速度会缓慢增加；松开后速度会快速回零，但不会瞬间归零。

## 键盘 SpeedL 模式

`easyarm_shell` 中输入 `speedl_teleop` 会进入键盘末端速度控制模式：

```text
easyarm> speedl_teleop
```

平移按键映射：

```text
w      -> y+
s      -> y-
a      -> x-
d      -> x+
Space  -> z+
c      -> z-
```

旋转按键映射：

```text
q          -> 绕 y 顺时针
e          -> 绕 y 逆时针
i          -> 俯仰向上，绕 x 顺时针
k          -> 俯仰向下
j          -> 偏航向左，绕 z 顺时针
l          -> 绕 z 逆时针
Esc        -> 退出并发送零速度
```

速度方向基于 `base_link` 坐标系，底层发布 MoveIt Servo 的 `TwistStamped` 输入。
