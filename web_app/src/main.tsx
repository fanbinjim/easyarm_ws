import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import {
  AlertTriangle,
  Activity,
  CircleStop,
  Gauge,
  ListChecks,
  Loader2,
  Lock,
  Pause,
  Play,
  Power,
  Radio,
  RefreshCw,
  Send,
  ShieldCheck,
  SlidersHorizontal,
  Wifi,
  WifiOff
} from "lucide-react";
import "./styles.css";

type ApiState = {
  success: boolean;
  message: string;
  mode: string;
  busy: boolean;
  active_task: string;
};

type BasicResponse = {
  success: boolean;
  message: string;
};

type JointResponse = {
  success: boolean;
  message: string;
  names: string[];
  positions: number[];
  velocities: number[];
  efforts: number[];
};

type PoseResponse = {
  success: boolean;
  message: string;
  frame_id: string;
  position: { x: number; y: number; z: number };
  orientation: { x: number; y: number; z: number; w: number };
};

type NamedStateResponse = {
  success: boolean;
  message: string;
  joint_names: string[];
  states: Array<{ name: string; positions: number[] }>;
};

type ControllerResponse = {
  success: boolean;
  message: string;
  controllers: Array<{ name: string; state: string; type: string }>;
};

type HealthResponse = {
  success: boolean;
  message: string;
  motion_server: Record<string, boolean>;
  controller_manager: boolean;
  joint_state_recent: boolean;
  is_mock_hardware: string;
  servo_state: string;
  trajectory_preview: string;
};

type Telemetry = {
  stamp: number;
  latest_joint_age_sec: number | null;
  active_action: {
    kind: string;
    state: string;
    accepted: boolean;
    done: boolean;
    success: boolean | null;
    message: string;
    feedback: string[];
    termination_reason?: string;
  };
  rosout: Array<{ level: number; name: string; message: string }>;
};

type ConfirmDialogState = {
  title: string;
  message: string;
  confirmLabel: string;
  tone: "danger" | "warn";
  onConfirm: () => void;
};

const JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"];
const DEFAULT_MOVEJ = [0, 1.85005, 2.68781, 0.9599, 1.57, 0];
const DEFAULT_MOVEL = { x: 0.25, y: 0, z: 0.25, qx: 0, qy: 0, qz: 0, qw: 1 };
const API_BASE_URL = (import.meta.env.VITE_EASYARM_API_BASE_URL ?? "").replace(/\/$/, "");

function apiUrl(path: string): string {
  return `${API_BASE_URL}${path}`;
}

function wsUrl(path: string, token: string): string {
  const tokenQuery = `token=${encodeURIComponent(token)}`;
  if (API_BASE_URL) {
    const url = new URL(path, API_BASE_URL);
    url.protocol = url.protocol === "https:" ? "wss:" : "ws:";
    url.search = tokenQuery;
    return url.toString();
  }
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return `${protocol}://${window.location.host}${path}?${tokenQuery}`;
}

function readToken(): string {
  return localStorage.getItem("easyarm_web_token") ?? "";
}

function writeToken(token: string) {
  localStorage.setItem("easyarm_web_token", token);
}

function numberText(value: unknown, digits = 4): string {
  return typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "n/a";
}

function isUnauthorizedError(err: unknown): boolean {
  return err instanceof Error && err.message.startsWith("401 ");
}

function parseHttpError(err: unknown): { status: number | null; detail: string } {
  if (!(err instanceof Error)) {
    return { status: null, detail: String(err) };
  }
  const match = err.message.match(/^(\d+)\s+(.+)$/s);
  if (!match) {
    return { status: null, detail: err.message };
  }
  const status = Number(match[1]);
  const body = match[2];
  try {
    const parsed = JSON.parse(body) as { detail?: unknown };
    return { status, detail: typeof parsed.detail === "string" ? parsed.detail : body };
  } catch {
    return { status, detail: body };
  }
}

function errorMessage(err: unknown): string {
  if (isUnauthorizedError(err)) {
    return "token 无效，请重新输入并保存。";
  }
  const { status, detail } = parseHttpError(err);
  if (detail.includes("service is not available") || detail.includes("action server is not available")) {
    return "机械臂服务未启动或已停止，请启动 EasyArm bringup 后刷新。";
  }
  return status == null ? detail : `${status} ${detail}`;
}

function recoverablePollMessage(err: unknown): string | null {
  const { status, detail } = parseHttpError(err);
  if (status === 504 || detail.includes("Timed out waiting for ROS response")) {
    return "ROS 响应超时，正在等待状态恢复，可稍后刷新。";
  }
  if (detail.includes("service is not available") || detail.includes("action server is not available")) {
    return "运动服务正在切换中，详情状态暂时不可用。";
  }
  return null;
}

function App() {
  const [token, setToken] = useState(readToken());
  const [draftToken, setDraftToken] = useState(token);
  const [bridgeReachable, setBridgeReachable] = useState(false);
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState("");
  const [pollWarning, setPollWarning] = useState("");
  const [authInvalid, setAuthInvalid] = useState(false);
  const [busyRequest, setBusyRequest] = useState(false);
  const [confirmDialog, setConfirmDialog] = useState<ConfirmDialogState | null>(null);
  const [shutdownRequested, setShutdownRequested] = useState(false);
  const [state, setState] = useState<ApiState | null>(null);
  const [joints, setJoints] = useState<JointResponse | null>(null);
  const [pose, setPose] = useState<PoseResponse | null>(null);
  const [namedStates, setNamedStates] = useState<NamedStateResponse | null>(null);
  const [controllers, setControllers] = useState<ControllerResponse | null>(null);
  const [health, setHealth] = useState<HealthResponse | null>(null);
  const [telemetry, setTelemetry] = useState<Telemetry | null>(null);
  const [planOnly, setPlanOnly] = useState(true);
  const [velocityScale, setVelocityScale] = useState(0.1);
  const [accelScale, setAccelScale] = useState(0.1);
  const [moveJ, setMoveJ] = useState(DEFAULT_MOVEJ);
  const [moveL, setMoveL] = useState(DEFAULT_MOVEL);
  const [selectedNamedState, setSelectedNamedState] = useState("");
  const [speedJ, setSpeedJ] = useState([0, 0, 0, 0, 0, 0]);
  const [speedL, setSpeedL] = useState([0, 0, 0, 0, 0, 0]);
  const [servoJ, setServoJ] = useState(DEFAULT_MOVEJ);
  const [servoL, setServoL] = useState(DEFAULT_MOVEL);
  const commandSocket = useRef<WebSocket | null>(null);
  const lastControllersRefresh = useRef(0);

  const api = useCallback(
    async <T,>(path: string, init?: RequestInit): Promise<T> => {
      const response = await fetch(apiUrl(path), {
        ...init,
        headers: {
          "Content-Type": "application/json",
          "X-EasyArm-Token": token,
          ...(init?.headers ?? {})
        }
      });
      if (!response.ok) {
        const text = await response.text();
        throw new Error(`${response.status} ${text}`);
      }
      return response.json() as Promise<T>;
    },
    [token]
  );

  const refresh = useCallback(async () => {
    if (!token || authInvalid) {
      return;
    }
    let healthData: HealthResponse;
    try {
      healthData = await api<HealthResponse>("/api/health");
      setBridgeReachable(true);
      setHealth(healthData);
    } catch (err) {
      setBridgeReachable(false);
      setPollWarning("");
      if (isUnauthorizedError(err)) {
        setAuthInvalid(true);
        setConnected(false);
      }
      setError(errorMessage(err));
      return;
    }

    const tasks: Array<Promise<void>> = [];

    if (healthData.motion_server?.get_state) {
      tasks.push(
        api<ApiState>("/api/state").then((stateData) => {
          setState(stateData);
          if (shutdownRequested && (healthData.joint_state_recent || stateData.success)) {
            setShutdownRequested(false);
          }
        })
      );
      tasks.push(api<JointResponse>("/api/joints").then((jointsData) => setJoints(jointsData)));
      tasks.push(api<PoseResponse>("/api/pose").then((poseData) => setPose(poseData)));
      tasks.push(
        api<NamedStateResponse>("/api/named-states").then((namedData) => {
          setNamedStates(namedData);
          if (!selectedNamedState && namedData.states.length > 0) {
            setSelectedNamedState(namedData.states[0].name);
          }
        })
      );
    }

    const now = Date.now();
    if (healthData.controller_manager && now - lastControllersRefresh.current > 5000) {
      tasks.push(
        api<ControllerResponse>("/api/controllers").then((controllerData) => {
          setControllers(controllerData);
          lastControllersRefresh.current = now;
        })
      );
    }

    if (tasks.length === 0) {
      setPollWarning("");
      setError("");
      return;
    }

    const results = await Promise.allSettled(tasks);
    let fatalError: unknown = null;
    let warningText = "";

    for (const result of results) {
      if (result.status !== "rejected") {
        continue;
      }
      if (isUnauthorizedError(result.reason)) {
        fatalError = result.reason;
        break;
      }
      const nextWarning = recoverablePollMessage(result.reason);
      if (nextWarning) {
        warningText ||= nextWarning;
        continue;
      }
      fatalError = result.reason;
      break;
    }

    if (fatalError) {
      setPollWarning("");
      if (isUnauthorizedError(fatalError)) {
        setAuthInvalid(true);
        setConnected(false);
      }
      setError(errorMessage(fatalError));
      return;
    }

    setPollWarning(warningText);
    setError("");
  }, [api, authInvalid, selectedNamedState, shutdownRequested, token]);

  useEffect(() => {
    refresh();
    const interval = window.setInterval(refresh, 500);
    return () => window.clearInterval(interval);
  }, [refresh]);

  useEffect(() => {
    if (!token || authInvalid) {
      setConnected(false);
      return;
    }
    const socket = new WebSocket(wsUrl("/ws/telemetry", token));
    socket.onopen = () => setConnected(true);
    socket.onclose = () => setConnected(false);
    socket.onerror = () => setConnected(false);
    socket.onmessage = (event) => setTelemetry(JSON.parse(event.data));
    return () => socket.close();
  }, [authInvalid, token]);

  const commandWs = useCallback(() => {
    if (commandSocket.current && commandSocket.current.readyState === WebSocket.OPEN) {
      return commandSocket.current;
    }
    const socket = new WebSocket(wsUrl("/ws/command-stream", token));
    commandSocket.current = socket;
    return socket;
  }, [token]);

  const sendStream = useCallback(
    (payload: unknown) => {
      const socket = commandWs();
      const send = () => socket.send(JSON.stringify(payload));
      if (socket.readyState === WebSocket.OPEN) {
        send();
      } else {
        socket.addEventListener("open", send, { once: true });
      }
    },
    [commandWs]
  );

  useEffect(() => {
    const halt = () => sendStream({ type: "halt" });
    const haltWhenHidden = () => {
      if (document.hidden) {
        halt();
      }
    };
    window.addEventListener("blur", halt);
    document.addEventListener("visibilitychange", haltWhenHidden);
    return () => {
      halt();
      window.removeEventListener("blur", halt);
      document.removeEventListener("visibilitychange", haltWhenHidden);
      commandSocket.current?.close();
    };
  }, [sendStream]);

  useEffect(() => {
    if (!confirmDialog) {
      return;
    }
    const closeWhenEsc = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        closeConfirm();
      }
    };
    window.addEventListener("keydown", closeWhenEsc);
    return () => window.removeEventListener("keydown", closeWhenEsc);
  }, [confirmDialog]);

  const submitToken = () => {
    const nextToken = draftToken.trim();
    writeToken(nextToken);
    setAuthInvalid(false);
    setBridgeReachable(false);
    setError("");
    setPollWarning("");
    setToken(nextToken);
  };

  const post = async <T,>(path: string, payload?: unknown): Promise<T> => {
    setBusyRequest(true);
    setError("");
    setPollWarning("");
    try {
      const response = await api<T>(path, {
        method: "POST",
        body: payload === undefined ? undefined : JSON.stringify(payload)
      });
      await refresh();
      return response;
    } catch (err) {
      if (isUnauthorizedError(err)) {
        setAuthInvalid(true);
        setConnected(false);
      }
      setError(errorMessage(err));
      throw err;
    } finally {
      setBusyRequest(false);
    }
  };

  const requestStop = async () => {
    const globalStopContext =
      state?.mode === "FREE_DRIVE" ||
      /^(Speed|Servo)/i.test(state?.active_task ?? "");

    try {
      const response = await post<BasicResponse>("/api/actions/active/cancel");
      if (response.success) {
        return;
      }
      if (response.message !== "No active action goal") {
        setPollWarning(response.message || "取消动作请求未生效，请稍后重试。");
        return;
      }
      if (!globalStopContext) {
        setPollWarning(activeActionInFlight ? "停止请求未命中当前动作，请稍后重试。" : "当前没有正在执行的动作。");
        return;
      }
    } catch (err) {
      const nextWarning = recoverablePollMessage(err);
      if (!nextWarning) {
        return;
      }
    }

    try {
      await post("/api/stop");
    } catch (err) {
      if (recoverablePollMessage(err)) {
        setError("");
        setPollWarning("Stop 请求已发出，ROS 响应超时，请观察状态卡片或稍后刷新。");
        void refresh();
      }
    }
  };

  const requestTopStop = async () => {
    try {
      await post("/api/stop");
    } catch (err) {
      if (recoverablePollMessage(err)) {
        setError("");
        setPollWarning("Stop request sent. ROS response timed out; watch the status cards and refresh soon.");
        void refresh();
      }
    }
  };

  const closeConfirm = () => setConfirmDialog(null);

  const openConfirm = (dialog: ConfirmDialogState) => {
    setConfirmDialog(dialog);
  };

  const requestSafeShutdown = () => {
    openConfirm({
      title: "安全关机",
      message: "确认执行安全关机？这会触发机械臂安全停机流程。",
      confirmLabel: "执行安全关机",
      tone: "danger",
      onConfirm: () => {
        setShutdownRequested(true);
        post("/api/safe-shutdown").catch(() => setShutdownRequested(false));
      }
    });
  };

  const enableExecuteMode = () => {
    if (!planOnly) {
      return;
    }
    openConfirm({
      title: "切换到执行模式",
      message: "执行模式下 MoveJ、MoveL 和 Named State 会下发真实运动。",
      confirmLabel: "切换到执行",
      tone: "danger",
      onConfirm: () => setPlanOnly(false)
    });
  };

  const requestModeChange = (mode: string) => {
    openConfirm({
      title: "切换控制模式",
      message: `确认切换到 ${mode} 模式？`,
      confirmLabel: `切换到 ${mode}`,
      tone: "warn",
      onConfirm: () => {
        void post("/api/set-mode", { mode }).catch(() => undefined);
      }
    });
  };

  const actionExecuteLabel = planOnly ? "规划" : "执行";

  const activeNamedValues = useMemo(() => {
    return namedStates?.states.find((item) => item.name === selectedNamedState)?.positions ?? [];
  }, [namedStates, selectedNamedState]);

  const activeAction = telemetry?.active_action;
  const backendTarget = API_BASE_URL || "Vite proxy";
  const bridgeStatusLabel = authInvalid ? "Auth required" : bridgeReachable ? "Bridge OK" : "Bridge Off";
  const bridgeStatusTone = bridgeReachable && !authInvalid ? "good" : "bad";
  const telemetryStatusLabel = connected ? "Telemetry" : "Telemetry Off";
  const telemetryStatusTone = connected ? "good" : "warn";
  const jointAge = telemetry?.latest_joint_age_sec;
  const jointStale = jointAge != null && jointAge > 2;
  const activeActionInFlight = Boolean(activeAction?.done === false && activeAction?.kind);
  const motionServiceOffline = health?.motion_server?.get_state === false;
  const serviceStopped = shutdownRequested || (bridgeReachable && motionServiceOffline);
  const jointAgeText = jointAge == null
    ? "n/a"
    : jointStale
      ? `stale ${jointAge.toFixed(1)}s`
      : `${jointAge.toFixed(2)}s`;
  const serviceStatusText = shutdownRequested
    ? "等待机械臂服务停止"
    : authInvalid
      ? "token 无效，已暂停自动刷新"
    : serviceStopped
      ? "机械臂服务未启动或已停止"
      : "服务运行中";
  const allCoreReady = Boolean(
    health?.controller_manager &&
    health?.joint_state_recent &&
    health?.motion_server?.movej &&
    health?.motion_server?.movel
  );
  const healthSummaryValue = authInvalid
    ? "Auth"
    : !bridgeReachable
      ? "Bridge Off"
      : serviceStopped
        ? "Motion Off"
        : allCoreReady && !jointStale && !pollWarning
          ? "Ready"
          : "Degraded";
  const healthSummaryDetail = authInvalid
    ? serviceStatusText
    : !bridgeReachable
      ? "Web bridge 未连接"
      : serviceStopped
        ? serviceStatusText
        : pollWarning
          ? pollWarning
          : health?.joint_state_recent === false
            ? "关节状态未更新"
            : `joint age ${jointAgeText}`;
  const healthSummaryTone = authInvalid || !bridgeReachable || serviceStopped
    ? "bad"
    : allCoreReady && !jointStale && !pollWarning
      ? "good"
      : "warn";

  return (
    <main className={`app-shell ${planOnly ? "plan-mode" : "execute-mode"}`}>
      <header className="command-header">
        <div>
          <div className="eyebrow">EasyArm A1</div>
          <h1>Web 调试台</h1>
        </div>
        <div className="topbar-actions">
          <StatusPill label={bridgeStatusLabel} tone={bridgeStatusTone} icon={bridgeReachable && !authInvalid ? <Wifi /> : <WifiOff />} />
          <StatusPill label={telemetryStatusLabel} tone={telemetryStatusTone} icon={connected ? <Wifi /> : <WifiOff />} />
          <StatusPill label={state?.mode ?? "UNKNOWN"} tone={state?.mode === "POSITION" ? "good" : "warn"} icon={<Gauge />} />
          <button
            className="danger-button"
            onClick={() => {
              void requestTopStop().catch(() => undefined);
            }}
          >
            <CircleStop /> {activeActionInFlight ? "取消动作" : "Stop"}
          </button>
          <button className="shutdown-button" disabled={busyRequest} onClick={requestSafeShutdown}>
            <Power /> 安全关机
          </button>
        </div>
      </header>

      <section className="connection-strip">
        <div className="connection-field">
          <Lock size={17} />
          <input value={draftToken} onChange={(event) => setDraftToken(event.target.value)} placeholder="web_token" type="password" />
          <button onClick={submitToken}>保存 token</button>
        </div>
        <div className="connection-meta">
          <span>{backendTarget}</span>
          <button className="ghost-button" onClick={refresh}>
            <RefreshCw /> 刷新
          </button>
        </div>
        {error && <span className={authInvalid ? "auth-text" : "error-text"}>{error}</span>}
        {(serviceStopped || authInvalid || pollWarning) && !error && (
          <span className="service-warning">{authInvalid ? serviceStatusText : pollWarning || serviceStatusText}</span>
        )}
      </section>

      <section className="summary-grid">
        <SummaryCard
          icon={<Activity />}
          label="系统"
          value={state?.busy ? "Busy" : "Idle"}
          detail={`task: ${state?.active_task || "idle"}`}
          tone={state?.busy ? "warn" : "good"}
        />
        <SummaryCard
          icon={<ShieldCheck />}
          label="健康"
          value={healthSummaryValue}
          detail={healthSummaryDetail}
          tone={healthSummaryTone}
        />
        <SummaryCard
          icon={<Loader2 />}
          label="动作"
          value={activeAction?.kind || "None"}
          detail={activeAction?.message || activeAction?.state || "idle"}
          tone={activeAction?.done === false ? "warn" : "neutral"}
        />
        <SummaryCard
          icon={<Gauge />}
          label="运动模式"
          value={planOnly ? "规划" : "执行"}
          detail={planOnly ? "不会下发真实运动" : "会下发真实运动"}
          tone={planOnly ? "good" : "bad"}
        />
      </section>

      <section className="workspace-grid">
        <div className="workspace-primary">
          <Panel title="运动控制台" icon={<Gauge />} className="control-console">
            <div className="control-head">
              <div className="mode-switch" role="group" aria-label="运动命令模式">
                <button type="button" className={planOnly ? "active" : ""} onClick={() => setPlanOnly(true)}>
                  规划
                </button>
                <button type="button" className={!planOnly ? "active execute" : ""} onClick={enableExecuteMode}>
                  执行
                </button>
              </div>
              <div className={`mode-note ${planOnly ? "safe" : "danger"}`}>
                {planOnly ? "当前只做 MoveIt 规划，不执行真实运动。" : "执行模式会下发真实运动，请确认机械臂环境安全。"}
              </div>
            </div>

            <div className="parameter-grid">
              <Range label="velocity" value={velocityScale} setValue={setVelocityScale} />
              <Range label="acceleration" value={accelScale} setValue={setAccelScale} />
              <div className="mode-buttons">
                {["POSITION", "IDLE", "FREE_DRIVE"].map((mode) => (
                  <button
                    key={mode}
                    className={state?.mode === mode ? "soft-active-button" : "ghost-button"}
                    onClick={() => requestModeChange(mode)}
                  >
                    {mode}
                  </button>
                ))}
              </div>
            </div>
          </Panel>

          <section className="motion-grid">
            <Panel title="MoveJ" icon={<Send />} className="motion-card">
              <NumberGrid values={moveJ} onChange={setMoveJ} labels={JOINT_NAMES} step={0.01} />
              <button
                disabled={busyRequest}
                onClick={() => {
                  void post("/api/movej", {
                    joints: moveJ,
                    velocity_scale: velocityScale,
                    acceleration_scale: accelScale,
                    execute: !planOnly
                  }).catch(() => undefined);
                }}
              >
                <Play /> {actionExecuteLabel} MoveJ
              </button>
            </Panel>

            <Panel title="MoveL" icon={<Send />} className="motion-card">
              <PoseEditor value={moveL} onChange={setMoveL} />
              <button
                disabled={busyRequest}
                onClick={() => {
                  void post("/api/movel", {
                    ...moveL,
                    frame_id: "base_link",
                    velocity_scale: velocityScale,
                    acceleration_scale: accelScale,
                    execute: !planOnly
                  }).catch(() => undefined);
                }}
              >
                <Play /> {actionExecuteLabel} MoveL
              </button>
            </Panel>

            <Panel title="Named State" icon={<ListChecks />} className="motion-card compact-motion-card">
              <select value={selectedNamedState} onChange={(event) => setSelectedNamedState(event.target.value)}>
                {(namedStates?.states ?? []).map((item) => (
                  <option key={item.name} value={item.name}>{item.name}</option>
                ))}
              </select>
              <div className="named-values">{activeNamedValues.map((value) => numberText(value)).join(" ") || "n/a"}</div>
              <button
                disabled={busyRequest || !selectedNamedState}
                onClick={() => {
                  void post("/api/move-named-state", {
                    name: selectedNamedState,
                    velocity_scale: velocityScale,
                    acceleration_scale: accelScale,
                    execute: !planOnly
                  }).catch(() => undefined);
                }}
              >
                <Play /> {actionExecuteLabel} Named State
              </button>
            </Panel>
          </section>

          <details className="panel stream-section">
            <summary>
              <span><Gauge /> 实时控制</span>
              <small>Speed / Servo 按住发送，松开 Halt</small>
            </summary>
            <div className="stream-grid">
              <div className="stream-card">
                <div className="subpanel-title">SpeedJ</div>
                <NumberGrid values={speedJ} onChange={setSpeedJ} labels={JOINT_NAMES} step={0.005} />
                <StreamButtons onStart={() => sendStream({ type: "speedj", velocities: speedJ })} onStop={() => sendStream({ type: "halt" })} />
              </div>

              <div className="stream-card">
                <div className="subpanel-title">SpeedL</div>
                <NumberGrid values={speedL} onChange={setSpeedL} labels={["vx", "vy", "vz", "wx", "wy", "wz"]} step={0.005} />
                <StreamButtons onStart={() => sendStream({ type: "speedl", twist: speedL, frame_id: "base_link" })} onStop={() => sendStream({ type: "halt" })} />
              </div>

              <div className="stream-card">
                <div className="subpanel-title">ServoJ</div>
                <NumberGrid values={servoJ} onChange={setServoJ} labels={JOINT_NAMES} step={0.01} />
                <StreamButtons onStart={() => sendStream({ type: "servoj", joints: servoJ })} onStop={() => sendStream({ type: "halt" })} />
              </div>

              <div className="stream-card">
                <div className="subpanel-title">ServoL</div>
                <PoseEditor value={servoL} onChange={setServoL} />
                <StreamButtons onStart={() => sendStream({ type: "servol", ...servoL, frame_id: "base_link" })} onStop={() => sendStream({ type: "halt" })} />
              </div>
            </div>
          </details>
        </div>

        <aside className="workspace-side">
          <Panel title="关节状态" icon={<SlidersHorizontal />}>
            <div className="joint-table">
              {JOINT_NAMES.map((name) => {
                const index = joints?.names.indexOf(name) ?? -1;
                return (
                  <div className="joint-row" key={name}>
                    <span>{name}</span>
                    <strong>{numberText(index >= 0 ? joints?.positions[index] : undefined)}</strong>
                    <small>{numberText(index >= 0 ? joints?.velocities[index] : undefined)} rad/s</small>
                  </div>
                );
              })}
            </div>
          </Panel>

          <Panel title="末端位姿" icon={<Radio />}>
            <Metric label="frame" value={pose?.frame_id ?? "base_link"} />
            <Metric label="x y z" value={`${numberText(pose?.position.x)} ${numberText(pose?.position.y)} ${numberText(pose?.position.z)}`} />
            <Metric label="qx qy qz qw" value={`${numberText(pose?.orientation.x)} ${numberText(pose?.orientation.y)} ${numberText(pose?.orientation.z)} ${numberText(pose?.orientation.w)}`} />
            <button className="ghost-button panel-action" onClick={() => pose && setServoL({ x: pose.position.x, y: pose.position.y, z: pose.position.z, qx: pose.orientation.x, qy: pose.orientation.y, qz: pose.orientation.z, qw: pose.orientation.w })}>
              复制到 ServoL
            </button>
          </Panel>

          <Panel title="动作反馈" icon={<Loader2 />}>
            <Metric label="kind" value={activeAction?.kind || "idle"} />
            <Metric label="state" value={activeAction?.state ?? "idle"} />
            <Metric label="message" value={activeAction?.message || "-"} />
            <div className="feedback-list">
              {(activeAction?.feedback ?? []).slice(-6).map((item, index) => (
                <span key={`${item}-${index}`}>{item}</span>
              ))}
            </div>
            <button
              className="ghost-button panel-action"
              onClick={() => {
                void post("/api/actions/active/cancel").catch(() => undefined);
              }}
            >
              取消当前 action
            </button>
          </Panel>

          <Panel title="Controllers" icon={<ListChecks />}>
            <div className="controller-list">
              {(controllers?.controllers ?? []).map((controller) => (
                <div className="controller-row" key={controller.name}>
                  <span>{controller.name}</span>
                  <strong>{controller.state}</strong>
                </div>
              ))}
            </div>
          </Panel>
        </aside>
      </section>

      <section className="log-panel">
        <div className="panel-title"><AlertTriangle /> ROS 日志</div>
        <div className="log-list">
          {(telemetry?.rosout ?? []).slice(-12).map((item, index) => (
            <div key={`${item.name}-${index}`}>
              <strong>{item.name}</strong>
              <span>{item.message}</span>
            </div>
          ))}
        </div>
      </section>

      {confirmDialog && (
        <div className="confirm-backdrop" role="presentation" onMouseDown={closeConfirm}>
          <div className="confirm-dialog" role="dialog" aria-modal="true" aria-labelledby="confirm-title" onMouseDown={(event) => event.stopPropagation()}>
            <div className={`confirm-icon ${confirmDialog.tone}`}>
              <AlertTriangle />
            </div>
            <div className="confirm-content">
              <h2 id="confirm-title">{confirmDialog.title}</h2>
              <p>{confirmDialog.message}</p>
              <div className="confirm-actions">
                <button className="ghost-button" onClick={closeConfirm}>取消</button>
                <button
                  className={confirmDialog.tone === "danger" ? "danger-button" : "soft-active-button"}
                  onClick={() => {
                    const action = confirmDialog.onConfirm;
                    closeConfirm();
                    action();
                  }}
                >
                  {confirmDialog.confirmLabel}
                </button>
              </div>
            </div>
          </div>
        </div>
      )}
    </main>
  );
}

function StatusPill({ label, tone, icon }: { label: string; tone: "good" | "warn" | "bad"; icon: React.ReactNode }) {
  return <span className={`status-pill ${tone}`}>{icon}{label}</span>;
}

function SummaryCard({
  icon,
  label,
  value,
  detail,
  tone
}: {
  icon: React.ReactNode;
  label: string;
  value: string;
  detail: string;
  tone: "good" | "warn" | "bad" | "neutral";
}) {
  return (
    <article className={`summary-card ${tone}`}>
      <div className="summary-icon">{icon}</div>
      <div>
        <span>{label}</span>
        <strong>{value}</strong>
        <small>{detail}</small>
      </div>
    </article>
  );
}

function Panel({
  title,
  icon,
  children,
  className = ""
}: {
  title: string;
  icon: React.ReactNode;
  children: React.ReactNode;
  className?: string;
}) {
  return (
    <section className={`panel ${className}`}>
      <div className="panel-title">{icon}{title}</div>
      {children}
    </section>
  );
}

function Metric({ label, value }: { label: string; value: string }) {
  return (
    <div className="metric">
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function Range({ label, value, setValue }: { label: string; value: number; setValue: (value: number) => void }) {
  return (
    <label className="range-row">
      <span>{label}</span>
      <input type="range" min="0.01" max="1" step="0.01" value={value} onChange={(event) => setValue(Number(event.target.value))} />
      <strong>{value.toFixed(2)}</strong>
    </label>
  );
}

function NumberGrid({
  values,
  onChange,
  labels,
  step
}: {
  values: number[];
  onChange: (value: number[]) => void;
  labels: string[];
  step: number;
}) {
  return (
    <div className="number-grid">
      {values.map((value, index) => (
        <label key={labels[index] ?? index}>
          <span>{labels[index] ?? index}</span>
          <input
            type="number"
            step={step}
            value={value}
            onChange={(event) => {
              const next = [...values];
              next[index] = Number(event.target.value);
              onChange(next);
            }}
          />
        </label>
      ))}
    </div>
  );
}

function PoseEditor({
  value,
  onChange
}: {
  value: typeof DEFAULT_MOVEL;
  onChange: (value: typeof DEFAULT_MOVEL) => void;
}) {
  const fields: Array<keyof typeof DEFAULT_MOVEL> = ["x", "y", "z", "qx", "qy", "qz", "qw"];
  return (
    <div className="number-grid pose-grid">
      {fields.map((field) => (
        <label key={field}>
          <span>{field}</span>
          <input
            type="number"
            step={field.startsWith("q") ? 0.01 : 0.005}
            value={value[field]}
            onChange={(event) => onChange({ ...value, [field]: Number(event.target.value) })}
          />
        </label>
      ))}
    </div>
  );
}

function StreamButtons({ onStart, onStop }: { onStart: () => void; onStop: () => void }) {
  const timerRef = useRef<number | null>(null);

  const start = () => {
    onStart();
    if (timerRef.current !== null) {
      window.clearInterval(timerRef.current);
    }
    timerRef.current = window.setInterval(onStart, 100);
  };

  const stop = () => {
    if (timerRef.current !== null) {
      window.clearInterval(timerRef.current);
      timerRef.current = null;
    }
    onStop();
  };

  return (
    <div className="stream-buttons">
      <button onMouseDown={start} onMouseUp={stop} onMouseLeave={stop} onTouchStart={start} onTouchEnd={stop}>
        <Play /> 按住发送
      </button>
      <button className="ghost-button" onClick={stop}>
        <Pause /> Halt
      </button>
    </div>
  );
}

const rootElement = document.getElementById("root");

if (!rootElement) {
  throw new Error("root element not found");
}

const app = <App />;

createRoot(rootElement).render(import.meta.env.DEV ? app : <React.StrictMode>{app}</React.StrictMode>);
