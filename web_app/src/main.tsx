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
  };
  rosout: Array<{ level: number; name: string; message: string }>;
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

function requireConfirm(message: string): boolean {
  return window.confirm(message);
}

function App() {
  const [token, setToken] = useState(readToken());
  const [draftToken, setDraftToken] = useState(token);
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState("");
  const [busyRequest, setBusyRequest] = useState(false);
  const [state, setState] = useState<ApiState | null>(null);
  const [joints, setJoints] = useState<JointResponse | null>(null);
  const [pose, setPose] = useState<PoseResponse | null>(null);
  const [namedStates, setNamedStates] = useState<NamedStateResponse | null>(null);
  const [controllers, setControllers] = useState<ControllerResponse | null>(null);
  const [health, setHealth] = useState<HealthResponse | null>(null);
  const [telemetry, setTelemetry] = useState<Telemetry | null>(null);
  const [planOnly, setPlanOnly] = useState(true);
  const [velocityScale, setVelocityScale] = useState(0.2);
  const [accelScale, setAccelScale] = useState(0.2);
  const [moveJ, setMoveJ] = useState(DEFAULT_MOVEJ);
  const [moveL, setMoveL] = useState(DEFAULT_MOVEL);
  const [selectedNamedState, setSelectedNamedState] = useState("");
  const [speedJ, setSpeedJ] = useState([0, 0, 0, 0, 0, 0]);
  const [speedL, setSpeedL] = useState([0, 0, 0, 0, 0, 0]);
  const [servoJ, setServoJ] = useState(DEFAULT_MOVEJ);
  const [servoL, setServoL] = useState(DEFAULT_MOVEL);
  const commandSocket = useRef<WebSocket | null>(null);

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
    if (!token) {
      return;
    }
    setError("");
    try {
      const [stateData, jointsData, poseData, namedData, controllersData, healthData] = await Promise.all([
        api<ApiState>("/api/state"),
        api<JointResponse>("/api/joints"),
        api<PoseResponse>("/api/pose"),
        api<NamedStateResponse>("/api/named-states"),
        api<ControllerResponse>("/api/controllers"),
        api<HealthResponse>("/api/health")
      ]);
      setState(stateData);
      setJoints(jointsData);
      setPose(poseData);
      setNamedStates(namedData);
      setControllers(controllersData);
      setHealth(healthData);
      if (!selectedNamedState && namedData.states.length > 0) {
        setSelectedNamedState(namedData.states[0].name);
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }, [api, selectedNamedState, token]);

  useEffect(() => {
    refresh();
    const interval = window.setInterval(refresh, 2000);
    return () => window.clearInterval(interval);
  }, [refresh]);

  useEffect(() => {
    if (!token) {
      setConnected(false);
      return;
    }
    const socket = new WebSocket(wsUrl("/ws/telemetry", token));
    socket.onopen = () => setConnected(true);
    socket.onclose = () => setConnected(false);
    socket.onerror = () => setConnected(false);
    socket.onmessage = (event) => setTelemetry(JSON.parse(event.data));
    return () => socket.close();
  }, [token]);

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

  const submitToken = () => {
    writeToken(draftToken.trim());
    setToken(draftToken.trim());
  };

  const post = async <T,>(path: string, payload?: unknown): Promise<T> => {
    setBusyRequest(true);
    setError("");
    try {
      const response = await api<T>(path, {
        method: "POST",
        body: payload === undefined ? undefined : JSON.stringify(payload)
      });
      await refresh();
      return response;
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
      throw err;
    } finally {
      setBusyRequest(false);
    }
  };

  const actionExecuteLabel = planOnly ? "规划" : "执行";

  const activeNamedValues = useMemo(() => {
    return namedStates?.states.find((item) => item.name === selectedNamedState)?.positions ?? [];
  }, [namedStates, selectedNamedState]);

  return (
    <main className="app-shell">
      <header className="topbar">
        <div>
          <div className="eyebrow">EasyArm A1</div>
          <h1>Web 调试台</h1>
        </div>
        <div className="topbar-actions">
          <StatusPill label={connected ? "Telemetry" : "Disconnected"} tone={connected ? "good" : "bad"} icon={connected ? <Wifi /> : <WifiOff />} />
          <StatusPill label={state?.mode ?? "UNKNOWN"} tone={state?.mode === "POSITION" ? "good" : "warn"} icon={<Gauge />} />
          <button className="danger-button" disabled={busyRequest} onClick={() => post("/api/stop")}>
            <CircleStop /> Stop
          </button>
        </div>
      </header>

      <section className="token-bar">
        <Lock size={17} />
        <input value={draftToken} onChange={(event) => setDraftToken(event.target.value)} placeholder="web_token" type="password" />
        <button onClick={submitToken}>保存 token</button>
        <button className="ghost-button" onClick={refresh}>
          <RefreshCw /> 刷新
        </button>
        {error && <span className="error-text">{error}</span>}
      </section>

      <section className="grid dashboard-grid">
        <Panel title="系统状态" icon={<Activity />}>
          <Metric label="mode" value={state?.mode ?? "UNKNOWN"} />
          <Metric label="busy" value={state?.busy ? "true" : "false"} />
          <Metric label="active task" value={state?.active_task || "idle"} />
          <Metric label="joint age" value={telemetry?.latest_joint_age_sec == null ? "n/a" : `${telemetry.latest_joint_age_sec.toFixed(2)}s`} />
          <Metric label="mock hardware" value={health?.is_mock_hardware ?? "unknown"} />
        </Panel>

        <Panel title="健康检查" icon={<ShieldCheck />}>
          <Metric label="controller manager" value={health?.controller_manager ? "ready" : "not ready"} />
          <Metric label="joint state" value={health?.joint_state_recent ? "recent" : "stale"} />
          <Metric label="movej" value={health?.motion_server?.movej ? "ready" : "not ready"} />
          <Metric label="movel" value={health?.motion_server?.movel ? "ready" : "not ready"} />
          <Metric label="reserved" value={`${health?.servo_state ?? "reserved"} / ${health?.trajectory_preview ?? "reserved"}`} />
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
      </section>

      <section className="grid main-grid">
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
          <button className="ghost-button" onClick={() => pose && setServoL({ x: pose.position.x, y: pose.position.y, z: pose.position.z, qx: pose.orientation.x, qy: pose.orientation.y, qz: pose.orientation.z, qw: pose.orientation.w })}>
            复制到 ServoL
          </button>
        </Panel>

        <Panel title="动作反馈" icon={<Loader2 />}>
          <Metric label="kind" value={telemetry?.active_action.kind || "idle"} />
          <Metric label="state" value={telemetry?.active_action.state ?? "idle"} />
          <Metric label="message" value={telemetry?.active_action.message || "-"} />
          <div className="feedback-list">
            {(telemetry?.active_action.feedback ?? []).slice(-6).map((item, index) => (
              <span key={`${item}-${index}`}>{item}</span>
            ))}
          </div>
          <button className="ghost-button" onClick={() => post("/api/actions/active/cancel")}>取消当前 action</button>
        </Panel>
      </section>

      <section className="control-layout">
        <Panel title="运动参数" icon={<Gauge />}>
          <label className="toggle-line">
            <input type="checkbox" checked={planOnly} onChange={(event) => setPlanOnly(event.target.checked)} />
            plan-only 默认开启
          </label>
          <Range label="velocity" value={velocityScale} setValue={setVelocityScale} />
          <Range label="acceleration" value={accelScale} setValue={setAccelScale} />
          <div className="mode-buttons">
            {["POSITION", "IDLE", "DRAG"].map((mode) => (
              <button
                key={mode}
                className="ghost-button"
                onClick={() => requireConfirm(`切换到 ${mode} 模式？`) && post("/api/set-mode", { mode })}
              >
                {mode}
              </button>
            ))}
          </div>
        </Panel>

        <Panel title="MoveJ" icon={<Send />}>
          <NumberGrid values={moveJ} onChange={setMoveJ} labels={JOINT_NAMES} step={0.01} />
          <button
            disabled={busyRequest}
            onClick={() =>
              (planOnly || requireConfirm("确认执行 MoveJ？")) &&
              post("/api/movej", { joints: moveJ, velocity_scale: velocityScale, acceleration_scale: accelScale, execute: !planOnly })
            }
          >
            <Play /> {actionExecuteLabel} MoveJ
          </button>
        </Panel>

        <Panel title="MoveL" icon={<Send />}>
          <PoseEditor value={moveL} onChange={setMoveL} />
          <button
            disabled={busyRequest}
            onClick={() =>
              (planOnly || requireConfirm("确认执行 MoveL？")) &&
              post("/api/movel", { ...moveL, frame_id: "base_link", velocity_scale: velocityScale, acceleration_scale: accelScale, execute: !planOnly })
            }
          >
            <Play /> {actionExecuteLabel} MoveL
          </button>
        </Panel>

        <Panel title="Named State" icon={<ListChecks />}>
          <select value={selectedNamedState} onChange={(event) => setSelectedNamedState(event.target.value)}>
            {(namedStates?.states ?? []).map((item) => (
              <option key={item.name} value={item.name}>{item.name}</option>
            ))}
          </select>
          <div className="named-values">{activeNamedValues.map((value) => numberText(value)).join(" ") || "n/a"}</div>
          <button
            disabled={busyRequest || !selectedNamedState}
            onClick={() =>
              (planOnly || requireConfirm(`确认运动到 ${selectedNamedState}？`)) &&
              post("/api/move-named-state", { name: selectedNamedState, velocity_scale: velocityScale, acceleration_scale: accelScale, execute: !planOnly })
            }
          >
            <Play /> {actionExecuteLabel} Named State
          </button>
        </Panel>
      </section>

      <section className="control-layout stream-layout">
        <Panel title="SpeedJ" icon={<Gauge />}>
          <NumberGrid values={speedJ} onChange={setSpeedJ} labels={JOINT_NAMES} step={0.005} />
          <StreamButtons onStart={() => sendStream({ type: "speedj", velocities: speedJ })} onStop={() => sendStream({ type: "halt" })} />
        </Panel>

        <Panel title="SpeedL" icon={<Gauge />}>
          <NumberGrid values={speedL} onChange={setSpeedL} labels={["vx", "vy", "vz", "wx", "wy", "wz"]} step={0.005} />
          <StreamButtons onStart={() => sendStream({ type: "speedl", twist: speedL, frame_id: "base_link" })} onStop={() => sendStream({ type: "halt" })} />
        </Panel>

        <Panel title="ServoJ" icon={<Gauge />}>
          <NumberGrid values={servoJ} onChange={setServoJ} labels={JOINT_NAMES} step={0.01} />
          <StreamButtons onStart={() => sendStream({ type: "servoj", joints: servoJ })} onStop={() => sendStream({ type: "halt" })} />
        </Panel>

        <Panel title="ServoL" icon={<Gauge />}>
          <PoseEditor value={servoL} onChange={setServoL} />
          <StreamButtons onStart={() => sendStream({ type: "servol", ...servoL, frame_id: "base_link" })} onStop={() => sendStream({ type: "halt" })} />
        </Panel>
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
    </main>
  );
}

function StatusPill({ label, tone, icon }: { label: string; tone: "good" | "warn" | "bad"; icon: React.ReactNode }) {
  return <span className={`status-pill ${tone}`}>{icon}{label}</span>;
}

function Panel({ title, icon, children }: { title: string; icon: React.ReactNode; children: React.ReactNode }) {
  return (
    <section className="panel">
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

createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
