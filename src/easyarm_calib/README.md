# easyarm_calib

EasyArm 相机采集与标定工具包，当前用于大恒 Galaxy 相机预览、棋盘格图像采集和相机内参标定。

## 工具

### camera_preview

打开第 1 台大恒 Galaxy 相机，实时预览并采集图片。

```bash
ros2 run easyarm_calib camera_preview
```

固定参数在 `easyarm_calib/camera_preview.py` 顶部修改：

```python
CAMERA_INDEX = 1
EXPOSURE_TIME_US = 10000.0
GAIN_DB = 10.0
FRAME_RATE_HZ = 30.0
MAX_PREVIEW_SIZE = 640
CAPTURE_ROOT = Path("data/camera_capture")
```

按键：

- `c` / `C`：保存当前图像
- `q` / `Q` / `Esc`：退出

预览图最长边会缩放到 `640` 以内，保存的 PNG 仍使用相机原始分辨率。

采集数据保存到 workspace 根目录下：

```text
data/camera_capture/YYYYMMDD/HHMMSS/IMG0001.png
data/camera_capture/YYYYMMDD/HHMMSS/IMG0002.png
```

例如：

```text
data/camera_capture/20260526/171222/IMG0001.png
```

### calibrate_camera

使用固定采集数据做棋盘格相机内参标定，并在完成后显示棋盘格角点可视化。

```bash
ros2 run easyarm_calib calibrate_camera
```

当前固定标定参数在 `easyarm_calib/calibrate_camera.py` 顶部：

```python
CAMERA_MODEL = "MER2-301-125U3M"
CAMERA_SERIAL = "FCZ21070977"
IMAGE_DIR = Path("data/camera_capture/20260526/171222")

CHESSBOARD_COLS = 11
CHESSBOARD_ROWS = 8
SQUARE_SIZE_M = 0.01
```

其中 `CHESSBOARD_COLS` 和 `CHESSBOARD_ROWS` 是棋盘格内角点数量，不是黑白格数量。当前标定板为 `11x8` 内角点，方格边长 `10mm`。

标定完成后会生成：

```text
data/camera_calibration/MER2-301-125U3M_FCZ21070977/20260526_171222/
```

输出内容：

- `camera_calibration.yaml`：ROS 常用相机内参格式
- `camera_calibration.json`：完整标定结果、成功/失败图片列表、每张图 reprojection error
- `corners_preview/`：每张图的棋盘格角点检测可视化
- `undistort_preview/`：部分图片的去畸变预览

本次数据 `data/camera_capture/20260526/171222` 的标定结果：

```text
valid images: 49 / 49
image size: 2048 x 1536
RMS reprojection error: 0.054848 px
```

内参：

```text
fx = 1204.86187381
fy = 1204.85455311
cx = 1018.50683942
cy = 757.862910784
```

畸变参数 `plumb_bob`：

```text
k1 = -0.100644739438
k2 = 0.0954016912757
p1 = 0.0006078501107
p2 = 0.0000868215706679
k3 = -0.0158470137521
```

### collect_joint_zero_vision

采集视觉零点标定数据：固定外部相机拍摄安装在 `Link6` 法兰上的棋盘格，同时保存当前 `/joint_states`。

```bash
ros2 run easyarm_calib collect_joint_zero_vision
```

固定参数：

- 相机：`MER2-301-125U3M (FCZ21070977)`
- 棋盘格：`11x8` 内角点，方格边长 `10mm`
- 相机内参：`data/camera_calibration/MER2-301-125U3M_FCZ21070977/20260526_171222/camera_calibration.yaml`
- 法兰到棋盘格表面初值：`Link6 +Z` 方向 `31.5mm`

按键：

- `c` / `C`：保存当前图像、角点和关节角，仅在角点和 `/joint_states` 都有效时保存
- `q` / `Q` / `Esc`：退出

输出目录：

```text
data/joint_zero_vision/YYYYMMDD/HHMMSS/
  images/IMG0001.png
  corners_preview/IMG0001.png
  samples.json
```

采集建议：

- 采集 `40~80` 组。
- `Joint2`、`Joint3`、`Joint4` 要有充分变化。
- `Joint1` 可取几个固定角度，例如 `-20 deg`、`0 deg`、`+20 deg`。
- `Joint5`、`Joint6` 可少量变化，第一版优化不放开它们。
- 棋盘格要覆盖图像中心、边缘、不同深度和不同倾角。

### optimize_joint_zero_vision

读取最近一次 `collect_joint_zero_vision` 数据，使用角点重投影误差优化关节零偏。

```bash
ros2 run easyarm_calib optimize_joint_zero_vision
```

第一版优化变量：

- `Joint2`、`Joint3`、`Joint4` 的零偏 `delta_q`
- `camera -> base` 外参
- `Link6 -> board` 外参

固定变量：

- 相机内参固定
- `Joint1`、`Joint5`、`Joint6` 零偏固定为 0

优化目标：

```text
u_detected = project(T_camera_base * FK_base_Link6(q + delta_q) * T_Link6_board * P_board)
```

输出目录：

```text
data/joint_zero_calibration/YYYYMMDD/HHMMSS/
  result.json
  joint_zero_offsets.yaml
  extrinsics.yaml
  reprojection_preview/
```

`joint_zero_offsets.yaml` 会给出建议写入 `easyarm_a1.ros2_control.xacro` 的 `position_offset`。程序不会自动修改 xacro。

重投影预览图中：

- 绿色点：实际检测到的棋盘格角点
- 红色十字：优化后模型投影点

## 大恒 Galaxy Python API

`camera_preview` 会优先从 workspace 内的参考目录加载 `gxipy`：

```text
ref/Galaxy_Linux_Python_*/Galaxy_Linux_Python_*/api
```

如果系统已经安装 Galaxy Python API，也可以直接使用系统安装的 `gxipy`。

注意：`gxipy` 只是 Python API。相机枚举还依赖大恒 Galaxy Linux SDK 底层驱动。

## 常见问题

### No Daheng Galaxy camera found

先确认系统能看到 USB 设备：

```bash
lsusb
```

如果 `lsusb` 看不到相机，问题在 USB 连接、虚拟机 USB 直通或 Galaxy SDK 驱动层，Python 程序无法枚举到相机。

VMware 中需要在 `Removable Devices` 里把大恒相机连接到虚拟机，并确认 USB Controller 使用 USB 3.x。

### 按键没有反应

`camera_preview` 同时读取 OpenCV 窗口和终端按键。若窗口按键无效，点击终端后按 `c` / `q`。

### 修改标定数据集

直接修改 `calibrate_camera.py` 顶部的 `IMAGE_DIR` 和 `OUTPUT_DIR`。如果换标定板，也要同步修改 `CHESSBOARD_COLS`、`CHESSBOARD_ROWS` 和 `SQUARE_SIZE_M`。

## 验证

语法检查：

```bash
python3 -m py_compile \
  src/easyarm_calib/easyarm_calib/camera_preview.py \
  src/easyarm_calib/easyarm_calib/calibrate_camera.py \
  src/easyarm_calib/easyarm_calib/collect_joint_zero_vision.py \
  src/easyarm_calib/easyarm_calib/optimize_joint_zero_vision.py \
  src/easyarm_calib/easyarm_calib/joint_zero_vision_common.py
```

构建包：

```bash
colcon build --packages-select easyarm_calib
```
