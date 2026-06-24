# 技术路线与架构

## 产品定位

`web_app` 是 EasyArm 的浏览器端调试控制台。当前第一目标不是替代 ROS/MoveIt 工具链，而是给操作者提供一个低摩擦的状态面板和安全运动入口：

- 快速查看机器人、controller、motion server 和 action 状态
- 通过 plan-only 验证 MoveJ、MoveL、Named State 目标
- 在明确确认后触发真实运动
- 提供 Stop、Cancel 和 stream halt 等安全控制
- 为后续桌面封装或局域网部署保留独立构建能力

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

前端只认识 `easyarm_web_bridge` 的 HTTP/WebSocket 契约，不直接依赖 ROS message 代码生成，也不直接访问 ROS graph。

## 当前实现

- 入口：`src/main.tsx`
- 样式：`src/styles.css`
- 构建：`vite.config.ts`、`tsconfig.json`
- 包管理：`package.json`、`package-lock.json`
- API base URL：
  - 开发时默认走 Vite proxy：`/api`、`/ws` -> `http://127.0.0.1:8000`
  - `EASYARM_WEB_BACKEND_URL` 控制 dev proxy 目标
  - `VITE_EASYARM_API_BASE_URL` 控制构建后直接访问的 backend URL
- 鉴权：
  - HTTP 使用 `X-EasyArm-Token`
  - WebSocket 使用 query string `token`
  - token 当前保存在 `localStorage.easyarm_web_token`

## 前端状态模型

当前页面将状态分为三类：

- 轮询状态：`state`、`joints`、`pose`、`namedStates`、`controllers`、`health`
- WebSocket 遥测：`telemetry.active_action`、`telemetry.rosout`、`latest_joint_age_sec`
- 本地表单状态：MoveJ、MoveL、Named State、SpeedJ、SpeedL、ServoJ、ServoL 参数，以及 `planOnly`

目前状态管理仍在单组件内，适合原型和早期联调。后续当 UI 拆分到多个视图时，应抽取：

- `src/api/`：HTTP/WS client 和 response types
- `src/features/status/`：状态面板
- `src/features/motion/`：MoveJ/MoveL/Named State
- `src/features/stream/`：Speed/Servo 流式控制
- `src/components/`：Panel、Metric、Range、NumberGrid 等通用控件

## 交互原则

- 默认 `planOnly=true`，避免误执行真实运动。
- 执行型命令需要确认：MoveJ、MoveL、Named State、Set Mode。
- `Stop` 始终放在顶层可见位置。
- stream 类命令采用按住发送、松开 halt；页面 blur、hidden 或卸载时发送 halt。
- 错误信息直接展示，但后续应按 HTTP/ROS 错误类型做更清晰分级。

## 视觉方向

EasyArm 控制台属于工业/调试工具，不做营销式页面。界面应保持信息密度、可扫描性和稳定布局：

- 优先使用表格、指标、工具栏、分组面板
- 控制按钮和危险操作视觉区分明确
- 数值输入标明单位或语义
- 适配 1366px 以上桌面优先，同时保持移动端可读

## 安全约束

- 前端不能成为硬件配置来源。
- 前端不能绕过 backend 的 token 校验。
- 前端不能假设当前连接的是 mock hardware；必须从 backend health 或显式用户上下文判断。
- 任何可能触发机器人运动的功能，都需要保留 plan-only、确认、停止和错误反馈路径。
