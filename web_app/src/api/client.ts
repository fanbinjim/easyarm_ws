import type {
  ActionResponse,
  CancelActionResponse,
  ControllerResponse,
  DebugDataResponse,
  DebugField,
  DebugLogsResponse,
  DebugStatusResponse,
  HealthResponse,
  JointResponse,
  NamedStateResponse,
  PoseResponse,
  RobotModelResponse,
  SafeShutdownResponse,
  StateResponse,
  StopResponse,
  StreamCommand,
  Telemetry,
} from "./types";

let _token = localStorage.getItem("easyarm_web_token") ?? "";
let _baseUrl = (import.meta.env.VITE_EASYARM_API_BASE_URL ?? "").replace(/\/$/, "");

export function getToken(): string {
  return _token;
}

export function setToken(token: string): void {
  _token = token;
  localStorage.setItem("easyarm_web_token", token);
}

export function getBaseUrl(): string {
  return _baseUrl;
}

export function setBaseUrl(url: string): void {
  _baseUrl = url.replace(/\/$/, "");
}

function apiUrl(path: string): string {
  return `${_baseUrl}${path}`;
}

/**
 * 用于 asset 加载（不支持 HTTP header 的 loader 用 query token）
 */
export function apiAssetUrl(path: string): string {
  const token = _token;
  if (/^https?:\/\//.test(path)) {
    const url = new URL(path);
    url.searchParams.set("token", token);
    return url.toString();
  }
  const base = _baseUrl || `${window.location.protocol}//${window.location.host}`;
  const sep = path.includes("?") ? "&" : "?";
  return `${base}${path}${sep}token=${encodeURIComponent(token)}`;
}

export async function apiText(path: string): Promise<string> {
  const response = await fetch(apiUrl(path), {
    headers: {
      "X-EasyArm-Token": _token,
    },
  });
  if (!response.ok) {
    const text = await response.text();
    const detail = parseErrorDetail(response.status, text);
    throw new ApiError(response.status, detail);
  }
  return response.text();
}

export function wsUrl(path: string): string {
  const tokenQuery = `token=${encodeURIComponent(_token)}`;
  if (_baseUrl) {
    const url = new URL(path, _baseUrl);
    url.protocol = url.protocol === "https:" ? "wss:" : "ws:";
    url.search = tokenQuery;
    return url.toString();
  }
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return `${protocol}://${window.location.host}${path}?${tokenQuery}`;
}

export class ApiError extends Error {
  constructor(
    public status: number | null,
    detail: string,
  ) {
    super(`${status != null ? `${status} ` : ""}${detail}`);
    this.name = "ApiError";
  }

  get isUnauthorized(): boolean {
    return this.status === 401;
  }

  get isServiceUnavailable(): boolean {
    return this.status === 503;
  }

  get isTimeout(): boolean {
    return this.status === 504;
  }

  get isBadRequest(): boolean {
    return this.status === 400;
  }

  get isRecoverable(): boolean {
    return this.isTimeout || this.isServiceUnavailable;
  }
}

export async function apiGet<T>(path: string): Promise<T> {
  const response = await fetch(apiUrl(path), {
    headers: {
      "Content-Type": "application/json",
      "X-EasyArm-Token": _token,
    },
  });
  if (!response.ok) {
    const text = await response.text();
    const detail = parseErrorDetail(response.status, text);
    throw new ApiError(response.status, detail);
  }
  return response.json() as Promise<T>;
}

export async function apiPost<T>(path: string, payload?: unknown): Promise<T> {
  const response = await fetch(apiUrl(path), {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      "X-EasyArm-Token": _token,
    },
    body: payload === undefined ? undefined : JSON.stringify(payload),
  });
  if (!response.ok) {
    const text = await response.text();
    const detail = parseErrorDetail(response.status, text);
    throw new ApiError(response.status, detail);
  }
  return response.json() as Promise<T>;
}

function parseErrorDetail(status: number, body: string): string {
  try {
    const parsed = JSON.parse(body) as { detail?: unknown };
    return typeof parsed.detail === "string" ? parsed.detail : body;
  } catch {
    return body || `HTTP ${status}`;
  }
}

export function errorMessage(err: unknown): string {
  if (err instanceof ApiError) {
    if (err.isUnauthorized) return "token 无效，请重新输入并保存。";
    const detail = err.message.replace(/^\d+\s+/, "");
    if (detail.includes("service is not available") || detail.includes("action server is not available")) {
      return "机械臂服务未启动或已停止，请启动 EasyArm bringup 后刷新。";
    }
    return detail;
  }
  if (err instanceof Error) return err.message;
  return String(err);
}

export function isRecoverableError(err: unknown): boolean {
  return err instanceof ApiError && err.isRecoverable;
}

export function recoverableMessage(err: unknown): string | null {
  if (!(err instanceof ApiError)) return null;
  if (err.isTimeout) return "ROS 响应超时，正在等待状态恢复，可稍后刷新。";
  if (err.isServiceUnavailable) return "运动服务正在切换中，详情状态暂时不可用。";
  return null;
}

export function numberText(value: unknown, digits = 4): string {
  return typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "n/a";
}

const _api = {
  get health(): Promise<HealthResponse> {
    return apiGet<HealthResponse>("/api/health");
  },
  get state(): Promise<StateResponse> {
    return apiGet<StateResponse>("/api/state");
  },
  get joints(): Promise<JointResponse> {
    return apiGet<JointResponse>("/api/joints");
  },
  get pose(): Promise<PoseResponse> {
    return apiGet<PoseResponse>("/api/pose");
  },
  get namedStates(): Promise<NamedStateResponse> {
    return apiGet<NamedStateResponse>("/api/named-states");
  },
  get controllers(): Promise<ControllerResponse> {
    return apiGet<ControllerResponse>("/api/controllers");
  },
  get robotModel(): Promise<RobotModelResponse> {
    return apiGet<RobotModelResponse>("/api/robot/model");
  },
  get debugStatus(): Promise<DebugStatusResponse> {
    return apiGet<DebugStatusResponse>("/api/debug/status");
  },
  get debugLogs(): Promise<DebugLogsResponse> {
    return apiGet<DebugLogsResponse>("/api/debug/logs");
  },
  debugStart(): Promise<DebugStatusResponse> {
    return apiPost<DebugStatusResponse>("/api/debug/start");
  },
  debugStop(): Promise<DebugStatusResponse> {
    return apiPost<DebugStatusResponse>("/api/debug/stop");
  },
  debugData(payload: {
    name: string;
    joint: number;
    field: DebugField;
    start: number;
    end: number;
    stride: number;
  }): Promise<DebugDataResponse> {
    const params = new URLSearchParams({
      joint: String(payload.joint),
      field: payload.field,
      start: String(payload.start),
      end: String(payload.end),
      stride: String(payload.stride),
    });
    return apiGet<DebugDataResponse>(`/api/debug/logs/${encodeURIComponent(payload.name)}/data?${params.toString()}`);
  },
  movej(payload: {
    joints: number[];
    velocity_scale?: number;
    acceleration_scale?: number;
    execute?: boolean;
  }): Promise<ActionResponse> {
    return apiPost<ActionResponse>("/api/movej", payload);
  },
  movel(payload: {
    x: number;
    y: number;
    z: number;
    qx: number;
    qy: number;
    qz: number;
    qw: number;
    frame_id?: string;
    velocity_scale?: number;
    acceleration_scale?: number;
    execute?: boolean;
  }): Promise<ActionResponse> {
    return apiPost<ActionResponse>("/api/movel", payload);
  },
  moveNamedState(payload: {
    name: string;
    velocity_scale?: number;
    acceleration_scale?: number;
    execute?: boolean;
  }): Promise<ActionResponse> {
    return apiPost<ActionResponse>("/api/move-named-state", payload);
  },
  cancelActiveAction(): Promise<CancelActionResponse> {
    return apiPost<CancelActionResponse>("/api/actions/active/cancel");
  },
  stop(): Promise<StopResponse> {
    return apiPost<StopResponse>("/api/stop");
  },
  safeShutdown(): Promise<SafeShutdownResponse> {
    return apiPost<SafeShutdownResponse>("/api/safe-shutdown");
  },
  setMode(mode: string): Promise<BasicResponse> {
    return apiPost<BasicResponse>("/api/set-mode", { mode });
  },
  createTelemetrySocket(onMessage: (telemetry: Telemetry) => void): TelemetrySocket {
    return new TelemetrySocket(onMessage);
  },
  createCommandSocket(): CommandSocket {
    return new CommandSocket();
  },
} as const;

export type BasicResponse = { success: boolean; message: string };

export class TelemetrySocket {
  private _ws: WebSocket | null = null;
  private _retryDelay = 1000;
  private _maxDelay = 30000;
  private _timer: ReturnType<typeof setTimeout> | null = null;
  private _intentional = false;

  constructor(private _onMessage: (telemetry: Telemetry) => void) {}

  get readyState(): number {
    return this._ws?.readyState ?? WebSocket.CLOSED;
  }

  connect(): void {
    this._intentional = false;
    this._doConnect();
  }

  disconnect(): void {
    this._intentional = true;
    if (this._timer) {
      clearTimeout(this._timer);
      this._timer = null;
    }
    this._ws?.close();
    this._ws = null;
  }

  private _doConnect(): void {
    const url = wsUrl("/ws/telemetry");
    const ws = new WebSocket(url);
    this._ws = ws;

    ws.onopen = () => {
      this._retryDelay = 1000;
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data) as Telemetry;
        this._onMessage(data);
      } catch {
        /* ignore malformed */
      }
    };

    ws.onclose = () => {
      if (this._intentional) return;
      this._scheduleReconnect();
    };

    ws.onerror = () => {
      /* onclose will fire after this */
    };
  }

  private _scheduleReconnect(): void {
    if (this._timer) return;
    this._timer = setTimeout(() => {
      this._timer = null;
      this._doConnect();
      this._retryDelay = Math.min(this._retryDelay * 2, this._maxDelay);
    }, this._retryDelay);
  }
}

export class CommandSocket {
  private _ws: WebSocket | null = null;
  private _pending: (() => void)[] = [];
  private _intentional = false;

  get readyState(): number {
    return this._ws?.readyState ?? WebSocket.CLOSED;
  }

  connect(): void {
    if (this._ws && this._ws.readyState === WebSocket.OPEN) return;
    this._intentional = false;
    this._ws = new WebSocket(wsUrl("/ws/command-stream"));
    const ws = this._ws;

    ws.onopen = () => {
      for (const cb of this._pending) cb();
      this._pending = [];
    };

    ws.onclose = () => {
      if (!this._intentional) {
        /* reconnect on next command */
        this._ws = null;
      }
    };
  }

  send(command: StreamCommand): void {
    const doSend = () => {
      if (this._ws?.readyState === WebSocket.OPEN) {
        this._ws.send(JSON.stringify(command));
      }
    };

    if (!this._ws || this._ws.readyState !== WebSocket.OPEN) {
      this.connect();
      this._pending.push(doSend);
    } else {
      doSend();
    }
  }

  close(): void {
    this._intentional = true;
    this._ws?.close();
    this._ws = null;
    this._pending = [];
  }
}

export const api = _api;
