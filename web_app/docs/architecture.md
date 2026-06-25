# 技术路线与架构

## 产品定位

`web_app` 是 EasyArm 的浏览器端调试控制台。第一屏就是操作台，提供机器人状态观察、运动命令编排、3D 可视化和安全控制。

## 运行拓扑

```text
Browser / Vite dev server
  |  fetch + WebSocket
  v
easyarm_web_bridge (FastAPI + ROS 2 node)
  |  ROS services/actions/topics
  v
easyarm_motion_server / controller_manager / ros2_control / hardware
```

前端只认识 `easyarm_web_bridge` 的 HTTP/WebSocket 契约。

## 目录结构

```
src/
├── main.tsx                        # 入口
├── App.tsx                         # 主布局编排
├── api/
│   ├── types.ts                    # 所有类型定义
│   └── client.ts                   # 统一 HTTP/WS 客户端
├── hooks/
│   ├── useSettings.ts              # token/URL 管理
│   ├── useHealth.ts                # health 轮询 (1.5s)
│   ├── useApiState.ts              # state/joints/pose/named-states/controllers 轮询
│   └── useTelemetry.ts             # WS telemetry + 自动重连 + 指数退避
├── components/
│   ├── StatusBar.tsx               # 顶部状态栏
│   ├── RobotViewer.tsx             # 3D 机器人查看器 (Three.js + urdf-loader)
│   ├── MotionPanel.tsx             # 运动控制台 (MoveJ/MoveL/Named State tabs)
│   ├── StreamPanel.tsx             # 流式控制 (Speed/Servo, 按住发送)
│   ├── JointTable.tsx              # 关节状态表 (rad/deg 切换)
│   ├── PosePanel.tsx               # 末端位姿
│   ├── ActionLog.tsx               # 动作反馈
│   ├── ControllerList.tsx           # 控制器列表
│   ├── RosLog.tsx                  # ROS 日志
│   ├── ConfirmDialog.tsx            # 二次确认弹窗
│   ├── SettingsDialog.tsx           # 设置面板
│   └── Toast.tsx                   # Toast 通知
├── ui/                             # 共享 UI 原语
│   ├── Panel.tsx
│   ├── Metric.tsx
│   ├── StatusPill.tsx
│   ├── SummaryCard.tsx
│   ├── NumberGrid.tsx
│   ├── PoseEditor.tsx
│   ├── Range.tsx
│   └── StreamButtons.tsx
└── styles.css                      # 全局样式
```

## 状态模型

### 后端连接状态
- `BackendStatus`: connected | disconnected | unauthorized | error

### Motion Server 状态
- `MotionServerStatus`: ready | unavailable | degraded

### Action 状态 (基于 telemetry active_action)
- `ActionState`: idle | sending | accepted | planning | executing | canceling | canceled | stopped | failed | done

### 遥测新鲜度
- `TelemetryFreshness`: fresh | stale | missing (stale > 3s)

### 数据流
- Health 轮询: 1.5s 间隔，驱动后端状态和 motion server 状态
- API 状态轮询: 1.5s 间隔，仅在 health.motion_server.get_state 为 true 时触发
- Telemetry WebSocket: 自动连接、断线自动重连（指数退避 1s→30s）
- Telemetry 断线 > 3s: 标记 stale，UI 显示 "Telemetry Stale"

## 安全约束
- 前端默认 planOnly=true，执行真实运动需二次确认
- Stop 按钮在 action 执行中自动调 cancel，非 action 运行中调 stop
- Safe Shutdown 独立按钮，二次确认含完整操作后果说明
- Stream 命令按住发送 / 松开 halt；页面 hidden/blur 时 halt
- 前端不能绕过 backend token 校验
- 前端不能假设当前连接的是 mock hardware

## 视觉方向
- 工程工具风格，安静专业，信息密度适中
- 危险操作三级视觉分级: Cancel < Stop < Safe Shutdown
- 按钮有 disabled/loading/error 状态
- 数值保留合理小数 (rad: 4 decimals, m: 4 decimals)

## Cookie 说明
- Token 存储在 localStorage.easyarm_web_token，修改后立即生效
- 开发时默认走 Vite proxy (/api, /ws -> http://127.0.0.1:8000)
- 构建后可通过 VITE_EASYARM_API_BASE_URL 设置 backend URL
