# easyarm_app

`easyarm_app` 是新的上层 app/CLI 层。它只调用 `easyarm_motion_server` 暴露的接口，不直接调用 MoveIt、不直接发底层 trajectory，也不直接访问硬件参数服务。

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
```

## 测试前启动

先启动 h0616 MoveIt 配置和 motion server。

Mock：

```bash
ros2 launch easyarm_a1_h0616_moveit_config demo.launch.py use_mock_hardware:=true
ros2 launch easyarm_motion_server h0616.launch.py use_mock_hardware:=true
```

真实硬件：

```bash
ros2 launch easyarm_a1_h0616_moveit_config demo.launch.py
ros2 launch easyarm_motion_server h0616.launch.py
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
