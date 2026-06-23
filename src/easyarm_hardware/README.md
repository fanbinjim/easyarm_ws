# easyarm_hardware

## 控制模式边界

`easyarm_hardware` 是 ros2_control `SystemInterface`，现在只保留硬件层必要模式：

- `IDLE`：纯阻尼模式，`kp=0`，`kd=idle_kd`，`velocity=0`，`tau=0`。
- `POSITION`：正常控制模式，服务 `MoveJ/MoveL` 的 JTC 链路，以及 controller 层 `FREE_DRIVE` / `SERVO` 链路。

旧 hardware `DRAG` 模式已经删除。拖拽请使用 controller 层入口：

```bash
ros2 run easyarm_app easyarm set-mode FREE_DRIVE
ros2 run easyarm_app easyarm set-mode POSITION
```

底层参数切换只用于硬件层调试：

```bash
ros2 param set /easyarm_hardware_control_mode controller_mode IDLE
ros2 param set /easyarm_hardware_control_mode controller_mode POSITION
```

查询当前请求模式：

```bash
ros2 param get /easyarm_hardware_control_mode controller_mode
```

`controller_mode` 参数表示 hardware 层请求模式；实际切换在下一次 hardware `write()` 中应用。`DRAG` 不再是合法 hardware mode。

相关硬件参数在 `src/easyarm_a1_moveit_config/config/easyarm_a1.ros2_control.xacro`：

```xml
<param name="hardware_control_mode">position</param>
<param name="gravity_compensation_scale">1.0</param>
<param name="idle_kd">4.0</param>
<param name="control_torque_limit_scale">1.0</param>
```

`urdf_path` 兼容字段暂时保留在 ros2_control xacro 中，但 hardware 生产链路不再用它加载动力学模型。启用重力补偿时，hardware 只从 `/robot_description` 获取 URDF XML；如果超时获取失败，configure 会失败。

`POSITION` 模式下 full command 来源由 ros2_control command interface 自动判断，不提供外部参数：

- `arm_controller` 未 claim `kp/kd` command interface 时，使用 hardware 内部 velocity feed-forward、kp/kd 和 `gravity(q)`。
- `easyarm_servo_controller` 或 `easyarm_freedrive_controller` claim `kp/kd` command interface 时，使用 controller 写入的 `kp/kd/effort`。
- `FREE_DRIVE` 的重力补偿和阻尼由 `easyarm_freedrive_controller` 输出完整 command，hardware 只负责透传、方向/offset、限幅和 CAN 发送。

## 250Hz 调试日志

`easyarm_hardware` 可选启用二进制调试日志。开启后，hardware `write()` 线程只把固定大小 sample 写入预分配 ring buffer，后台线程批量落盘；ring buffer 满时丢日志并计数，不阻塞 CAN 发送。

硬件参数在 `src/easyarm_a1_moveit_config/config/easyarm_a1.ros2_control.xacro`：

```xml
<param name="debug_enable">${debug_enable}</param>
<param name="debug_buffer_seconds">60</param>
```

默认关闭。启动 demo 时开启日志：

```bash
ros2 launch easyarm_a1_moveit_config demo.launch.py debug_enable:=true
```

日志默认写到 `/dev/shm/easyarm_log_YYYYMMDD_HHMMSS.bin`，减少磁盘写入抖动。停止 hardware 时日志线程会 flush 并打印 `written`/`dropped` 统计。

解析日志：

```bash
python3 src/easyarm_hardware/scripts/decode_debug_log.py
```

默认读取 `/dev/shm/easyarm_log_*.bin` 中最新的日志，输出根目录为 `debug/plot/`。输出文件夹名来自输入 bin 文件名，例如 `/dev/shm/easyarm_log_20260613_125513.bin` 会输出 CSV 到 `debug/plot/easyarm_log_20260613_125513/easyarm_log_20260613_125513.csv`，输出图片到 `debug/plot/easyarm_log_20260613_125513/all_all/all.png`。

如果默认路径下还没有日志文件，脚本会提示错误；也可以直接把 `.bin` 文件路径作为第一个参数传入。

只绘制指定时间窗口，单位为秒：

```bash
python3 src/easyarm_hardware/scripts/decode_debug_log.py \
  --start 2.0 \
  --end 6.0
```

上面的图片会保存到对应日志文件夹的 `2_0_6_0/all.png`；如果只指定一侧时间，另一侧文件夹名使用 `all`。

同时拆分保存每个子图：

```bash
python3 src/easyarm_hardware/scripts/decode_debug_log.py \
  --start 2.0 \
  --end 6.0 \
  --split=true
```

组合大图会包含每类对比图，以及对比图中每条曲线的单独图。拆分图片默认保存到对应时间窗口目录的 `split/` 下，文件名来自图标题，例如 `Joint_1_position_error.png`、`Joint_1_position_error_command_-_state.png`。

## TODO

- `IDLE` 和 `POSITION` 仍直接写在 `easyarm_hardware` 中；长期目标是让 hardware 只保留硬件读写、安全限幅和底层状态同步。
- `SERVO` 和 `FREE_DRIVE` 已经由 `easyarm_controller` 写入完整 command：`position + velocity + kp + kd + effort`。
- `MoveJ/MoveL` 仍通过 JTC 进入 hardware `POSITION` 链路，hardware 内部重力补偿暂时保留以服务 MOVE。
- 后续继续把重力补偿、阻尼控制、速度/加速度前馈逐步上提到 controller 层。
