# EasyArm Web App

EasyArm Web App 是 EasyArm 项目的前端控制台，当前定位是面向调试、联调和运行状态观察的 React/Vite 应用。它不属于 ROS 2 package，不直接访问硬件，也不承载机器人控制逻辑；所有机器人状态查询、运动命令和流式控制都必须通过 `easyarm_web_bridge` 暴露的 HTTP/WebSocket 接口完成。

## 前端边界

- 只修改 `web_app/` 内文件；仓库中 ROS 2 package、launch、URDF、MoveIt 配置、硬件参数等只能只读参考。
- 不在前端保存或推导 motor ID、CAN 参数、zero offset、joint limit、control gain、`use_mock_hardware` 默认值等硬件安全配置。
- 默认以 plan-only 作为运动命令的安全入口；执行真实运动前必须由界面显式确认。
- `Stop`、`Cancel active action`、窗口失焦/隐藏时的 stream halt 属于安全交互，后续改动不能弱化这些行为。
- 接口字段以 `easyarm_web_bridge` 当前实现为准；本文档只记录前端依赖的契约。

## 技术栈

- React 18 + TypeScript strict mode
- Vite 5
- `lucide-react` 图标
- 原生 `fetch` + `WebSocket`
- CSS 文件级样式，目前尚未引入组件库、状态管理库或测试框架

## 当前进展

- 已有单页控制台入口：`src/main.tsx`
- 已支持 token 保存、本地 API base URL 配置、Vite dev proxy
- 已支持状态轮询：`/api/state`、`/api/joints`、`/api/pose`、`/api/named-states`、`/api/controllers`、`/api/health`
- 已支持遥测 WebSocket：`/ws/telemetry`
- 已支持运动命令：MoveJ、MoveL、MoveNamedState、Stop、Cancel active action
- 顶层红色 `Stop` 已调整为动态语义：动作执行中优先取消当前 active action；空闲、`FREE_DRIVE` 或 Servo/Speed 场景才回落到 `/api/stop`
- 已支持流式命令通道：SpeedJ、SpeedL、ServoJ、ServoL、Halt
- 已知问题：`src/main.tsx` 中部分中文文案存在编码显示异常，需要在后续 UI 整理中统一修复

## 本地开发

安装前端依赖：

```bash
cd web_app
npm install
```

启动 ROS backend：

```bash
cd ~/easyarm_ws
source install/setup.bash
export EASYARM_WEB_TOKEN=easyarm
ros2 launch easyarm_web_bridge web_bridge.launch.py
```

启动前端：

```bash
cd web_app
npm run dev -- --host 127.0.0.1 --port 5173
```

打开 `http://127.0.0.1:5173`，输入与 backend 一致的 token。

默认情况下，Vite 会把 `/api` 和 `/ws` 代理到 `http://127.0.0.1:8000`。如需连接其他 backend：

```bash
EASYARM_WEB_BACKEND_URL=http://192.168.1.20:8000 npm run dev -- --host 0.0.0.0
```

构建独立产物时，如需让页面直接访问指定 backend：

```bash
VITE_EASYARM_API_BASE_URL=http://192.168.1.20:8000 npm run build
```

## 常用命令

```bash
npm run dev
npm run build
npm run preview
```

`npm run build` 会先执行 `tsc --noEmit`，再执行 `vite build`。

## 文档索引

- [前端协作规则](./AGENTS.md)
- [技术路线与架构](./docs/architecture.md)
- [接口文档](./docs/api.md)
- [开发计划与技术债](./docs/roadmap.md)
