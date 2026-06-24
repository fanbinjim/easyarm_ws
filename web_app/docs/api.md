# EasyArm Web API 文档

本文档记录 `web_app` 当前依赖的 `easyarm_web_bridge` HTTP/WebSocket 契约。字段以 backend 当前实现为准；前端新增依赖时必须同步更新本文档。

## 基础约定

- 默认 base URL：`http://127.0.0.1:8000`
- Vite dev proxy：前端请求 `/api/*` 和 `/ws/*` 即可
- HTTP 鉴权 header：`X-EasyArm-Token: <token>`
- HTTP 也支持 `Authorization: Bearer <token>` 或 query `?token=<token>`，但前端默认使用 header
- WebSocket 鉴权：`/ws/<name>?token=<token>`
- HTTP 成功返回 JSON；token 错误返回 `401`；backend 未配置 token 返回 `503`
- ROS service/action 超时通常返回 `504`；参数或 ROS 可用性错误通常返回 `400`

## 通用类型

```ts
type BasicResponse = {
  success: boolean;
  message: string;
};

type Pose = {
  frame_id: string;
  position: { x: number; y: number; z: number };
  orientation: { x: number; y: number; z: number; w: number };
};
```

## GET /api/health

用途：读取 bridge 与关键 ROS 服务/action 的健康状态。

Response:

```ts
type HealthResponse = BasicResponse & {
  motion_server: {
    get_state: boolean;
    movej: boolean;
    movel: boolean;
    move_named_state: boolean;
  };
  controller_manager: boolean;
  joint_state_recent: boolean;
  is_mock_hardware: string;
  servo_state: string;
  trajectory_preview: string;
};
```

备注：`is_mock_hardware`、`servo_state`、`trajectory_preview` 当前可能是 `"unknown"` 或 `"reserved"`。

## GET /api/state

用途：读取 EasyArm motion/control 状态。

Response:

```ts
type StateResponse = BasicResponse & {
  mode: string;
  busy: boolean;
  active_task: string;
};
```

## GET /api/joints

用途：读取当前关节状态。

Response:

```ts
type JointResponse = BasicResponse & {
  names: string[];
  positions: number[];
  velocities: number[];
  efforts: number[];
};
```

单位约定：`positions` 为 `rad`，`velocities` 为 `rad/s`，`efforts` 通常为 `Nm` 或底层 controller 提供的 effort 单位。

## GET /api/pose

用途：读取末端位姿。

Query:

```ts
type PoseQuery = {
  target_frame?: string;
  source_frame?: string;
};
```

Response:

```ts
type PoseResponse = BasicResponse & Pose;
```

## GET /api/named-states

用途：读取 MoveIt named states。

Response:

```ts
type NamedStateResponse = BasicResponse & {
  joint_names: string[];
  states: Array<{
    name: string;
    positions: number[];
  }>;
};
```

## GET /api/controllers

用途：读取 controller manager 的 controller 列表。

Response:

```ts
type ControllerResponse = BasicResponse & {
  controllers: Array<{
    name: string;
    state: string;
    type: string;
    claimed_interfaces: string[];
    required_command_interfaces: string[];
    required_state_interfaces: string[];
  }>;
};
```

## POST /api/set-mode

用途：切换控制模式。

Request:

```ts
type SetModeRequest = {
  mode: "POSITION" | "IDLE" | "FREE_DRIVE";
};
```

Response:

```ts
type SetModeResponse = BasicResponse;
```

前端安全要求：该操作需要用户确认。

备注：`DRAG` 已废弃，前端不得再发送；拖拽/自由拖动入口统一使用 `FREE_DRIVE`。

## POST /api/stop

用途：调用 `/easyarm/stop`。

Request：无 body。

Response:

```ts
type StopResponse = BasicResponse;
```

前端安全要求：顶层常驻可见。

前端交互约定：

- 当页面检测到有 active action 正在执行时，顶层红色停止按钮应优先调用 `POST /api/actions/active/cancel`，避免把普通“停止当前动作”升级成全局 stop。
- `POST /api/stop` 主要保留给 `FREE_DRIVE`、Servo/Speed 流式控制退出，以及没有 active action goal 时的全局停止。

## POST /api/safe-shutdown

用途：触发 EasyArm 安全关机流程。该接口由 backend 负责实现具体步骤，例如停止当前运动、切换到安全控制模式、运动到 ready 位、停用硬件或结束 launch。

Request：无 body。

Response:

```ts
type SafeShutdownResponse = BasicResponse;
```

前端安全要求：

- 顶层可见，但必须与普通 `Stop` 区分。
- 点击前必须二次确认。
- 失败时展示 backend 返回的错误。
- 该接口属于高风险真实硬件操作，不能由前端自行模拟 shutdown 步骤。

## POST /api/actions/active/cancel

用途：取消当前 active action goal。

Request：无 body。

Response:

```ts
type CancelActionResponse = BasicResponse;
```

## POST /api/movej

用途：发送 MoveJ action。

Request:

```ts
type MoveJRequest = {
  joints: number[]; // length 6, rad
  velocity_scale?: number;
  acceleration_scale?: number;
  execute?: boolean; // false 表示只规划
};
```

Response:

```ts
type ActionResponse = BasicResponse & {
  accepted: boolean;
  feedback: string[];
};
```

前端安全要求：默认 `execute=false`；当 `execute=true` 时需要用户确认。

## POST /api/movel

用途：发送 MoveL action。

Request 支持扁平结构：

```ts
type MoveLRequest = {
  x: number;
  y: number;
  z: number;
  qx: number;
  qy: number;
  qz: number;
  qw: number;
  frame_id?: string; // 默认 base_link
  velocity_scale?: number;
  acceleration_scale?: number;
  execute?: boolean;
};
```

也支持嵌套 pose：

```ts
type MoveLNestedRequest = {
  frame_id?: string;
  pose: {
    frame_id?: string;
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
  velocity_scale?: number;
  acceleration_scale?: number;
  execute?: boolean;
};
```

Response:

```ts
type MoveLResponse = ActionResponse;
```

前端安全要求：默认 `execute=false`；当 `execute=true` 时需要用户确认。

## POST /api/move-named-state

用途：发送 MoveNamedState action。

Request:

```ts
type MoveNamedStateRequest = {
  name: string;
  velocity_scale?: number;
  acceleration_scale?: number;
  execute?: boolean;
};
```

Response:

```ts
type MoveNamedStateResponse = ActionResponse;
```

前端安全要求：默认 `execute=false`；当 `execute=true` 时需要用户确认。

## WS /ws/telemetry

用途：订阅遥测快照。backend 当前约每 `0.5s` 推送一次。

URL:

```text
/ws/telemetry?token=<token>
```

Message:

```ts
type TelemetryMessage = {
  stamp: number;
  latest_joints: null | {
    names: string[];
    positions: number[];
    velocities: number[];
    efforts: number[];
    stamp: { sec: number; nanosec: number };
  };
  latest_joint_age_sec: number | null;
  active_action: {
    kind: string;
    state: string;
    accepted: boolean;
    done: boolean;
    success: boolean | null;
    message: string;
    feedback: string[];
  };
  rosout: Array<{
    stamp: { sec: number; nanosec: number };
    level: number;
    name: string;
    message: string;
  }>;
};
```

## WS /ws/command-stream

用途：发送流式控制命令。backend 收到每条命令后会返回 `{ success, message }`。

URL:

```text
/ws/command-stream?token=<token>
```

Command:

```ts
type StreamCommand =
  | { type: "speedj"; velocities: number[] } // length 6, rad/s
  | { type: "speedl"; twist: number[]; frame_id?: string } // [vx, vy, vz, wx, wy, wz]
  | { type: "servoj"; joints: number[] } // length 6, rad
  | ({ type: "servol"; frame_id?: string } & (
      | { x: number; y: number; z: number; qx: number; qy: number; qz: number; qw: number }
      | { pose: Pose }
    ))
  | { type: "halt" };
```

前端安全要求：

- stream 命令应采用按住发送或其他 dead-man 交互。
- 停止交互、鼠标离开、触摸结束、窗口失焦、页面隐藏或组件卸载时发送 `{ type: "halt" }`。
- WebSocket 断开时 backend 会尝试 publish halt 并调用 stop；前端仍应主动 halt。
