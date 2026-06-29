# easyarm_task

`easyarm_task` 放机械臂应用任务。当前包含球平衡视觉任务骨架：

```bash
ros2 run easyarm_task easyarm_ball_balance
```

启动 RealSense：

```bash
ros2 launch easyarm_a1_bringup bringup.launch.py camera:=true
```

默认订阅 `/camera/camera/color/image_raw`，按 `640x480` 绘制 OpenCV
预览窗口，并检测白色圆盘和黑色圆形物体。窗口中：

- 红色框：检测到的圆盘区域。
- 绿色框：检测到的黑色圆形物体。
- `offset`：黑色物体相对圆盘中心的归一化偏移，圆盘半径为 `1.0`。

常用调试参数：

```bash
ros2 run easyarm_task easyarm_ball_balance \
  --image-topic /camera/camera/color/image_raw \
  --width 640 \
  --height 480
```

检测阈值后续可以按现场光照调整：

```bash
ros2 run easyarm_task easyarm_ball_balance \
  --plate-min-area 12000 \
  --ball-max-value 85 \
  --debug-mask
```

`--debug-mask` 会显示：

- `01_plate_raw_mask`：白色阈值 mask。
- `02_plate_roi_mask`：圆盘搜索 ROI。
- `03_plate_roi_applied_mask`：白色阈值和 ROI 相交结果。
- `04_plate_clean_mask`：颜色连通域检测使用的 mask。
- `05_plate_edge_mask`：ROI 内 Canny 边沿。
- `06_ball_mask`：黑色圆形物体 mask。
- `07_plate_candidates`：颜色候选和边沿圆候选，以及淘汰原因。

圆盘检测现在同时使用颜色区域和边沿圆候选。边沿分支对背景颜色更宽容，
主要看圆盘外圆边沿、圆内白色比例和 ROI 位置：

```bash
ros2 run easyarm_task easyarm_ball_balance \
  --plate-edge-min-support 0.16 \
  --plate-edge-min-white-ratio 0.35 \
  --debug-mask
```

圆盘默认只在画面中下部 ROI 内搜索，并默认关闭 Hough 圆检测，避免把桌面、
线缆或机械零件误判成大圆。ROI 可按现场相机位置调整：

```bash
ros2 run easyarm_task easyarm_ball_balance \
  --plate-roi-x-min 0.30 \
  --plate-roi-x-max 0.90 \
  --plate-roi-y-min 0.38 \
  --plate-roi-y-max 0.96
```

如果圆盘边缘清晰但白色阈值效果不好，可以临时打开 Hough 兜底：

```bash
ros2 run easyarm_task easyarm_ball_balance --enable-hough-plate
```
