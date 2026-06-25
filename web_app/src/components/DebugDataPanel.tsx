import { useCallback, useEffect, useMemo, useState } from "react";
import { BarChart3, Download, RefreshCw, Square, Play } from "lucide-react";
import { api, apiAssetUrl, ApiError, errorMessage, numberText } from "../api/client";
import type { DebugDataPoint, DebugField, DebugLogEntry, DebugStatusResponse } from "../api/types";
import { Panel } from "../ui/Panel";
import { Metric } from "../ui/Metric";

const DEBUG_FIELDS: DebugField[] = [
  "position_error",
  "smoothed_position_error",
  "velocity_error",
  "motor_velocity_error",
  "command_position",
  "state_position",
  "command_velocity",
  "state_velocity",
  "motor_torque",
  "write_duration_us",
];

const JOINTS = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"];

type DebugDataPanelProps = {
  token: string;
};

export function DebugDataPanel({ token }: DebugDataPanelProps) {
  const [status, setStatus] = useState<DebugStatusResponse | null>(null);
  const [logs, setLogs] = useState<DebugLogEntry[]>([]);
  const [selectedLog, setSelectedLog] = useState("");
  const [joint, setJoint] = useState(0);
  const [field, setField] = useState<DebugField>("position_error");
  const [start, setStart] = useState(0);
  const [end, setEnd] = useState(10);
  const [stride, setStride] = useState(5);
  const [points, setPoints] = useState<DebugDataPoint[]>([]);
  const [statusLoading, setStatusLoading] = useState(false);
  const [recordLoading, setRecordLoading] = useState<"start" | "stop" | "">("");
  const [logsLoading, setLogsLoading] = useState(false);
  const [dataLoading, setDataLoading] = useState(false);
  const [error, setError] = useState("");
  const [debugApiUnavailable, setDebugApiUnavailable] = useState(false);

  const selectedLogEntry = useMemo(() => logs.find((item) => item.name === selectedLog) ?? null, [logs, selectedLog]);

  const handleRequestError = useCallback((err: unknown) => {
    if (err instanceof ApiError && err.status === 404) {
      setDebugApiUnavailable(true);
      setError("");
      return;
    }
    if (err instanceof ApiError && err.status != null && err.status >= 500) {
      setError("");
      return;
    }
    setError(errorMessage(err));
  }, []);

  const refreshStatus = useCallback(async () => {
    if (!token) return;
    setStatusLoading(true);
    try {
      setStatus(await api.debugStatus);
      setDebugApiUnavailable(false);
      setError("");
    } catch (err) {
      handleRequestError(err);
    } finally {
      setStatusLoading(false);
    }
  }, [handleRequestError, token]);

  const refreshLogs = useCallback(async () => {
    if (!token) return;
    setLogsLoading(true);
    try {
      const response = await api.debugLogs;
      setDebugApiUnavailable(false);
      setError("");
      setLogs(response.logs);
      if (response.logs.length > 0) {
        setSelectedLog((current) => current && response.logs.some((log) => log.name === current) ? current : response.logs[0].name);
      } else {
        setSelectedLog("");
        setPoints([]);
      }
    } catch (err) {
      handleRequestError(err);
    } finally {
      setLogsLoading(false);
    }
  }, [handleRequestError, token]);

  useEffect(() => {
    if (!token) return;
    void refreshStatus();
    void refreshLogs();
  }, [refreshLogs, refreshStatus, token]);

  useEffect(() => {
    if (!token || debugApiUnavailable) return;
    const timer = window.setInterval(() => {
      void refreshStatus();
    }, 1500);
    return () => window.clearInterval(timer);
  }, [debugApiUnavailable, refreshStatus, token]);

  const startRecording = async () => {
    if (!token || debugApiUnavailable) return;
    setRecordLoading("start");
    setError("");
    try {
      setStatus(await api.debugStart());
      await refreshLogs();
    } catch (err) {
      handleRequestError(err);
    } finally {
      setRecordLoading("");
    }
  };

  const stopRecording = async () => {
    if (!token || debugApiUnavailable) return;
    setRecordLoading("stop");
    setError("");
    try {
      setStatus(await api.debugStop());
      await refreshLogs();
    } catch (err) {
      handleRequestError(err);
    } finally {
      setRecordLoading("");
    }
  };

  const loadData = async () => {
    if (!token || debugApiUnavailable || !selectedLog) return;
    setDataLoading(true);
    setError("");
    try {
      const response = await api.debugData({ name: selectedLog, joint, field, start, end, stride });
      setPoints(response.points);
    } catch (err) {
      setPoints([]);
      handleRequestError(err);
    } finally {
      setDataLoading(false);
    }
  };

  const downloadHref = selectedLog ? apiAssetUrl(`/api/debug/logs/${encodeURIComponent(selectedLog)}/download`) : "";

  return (
    <Panel title="Debug / Data Analysis" icon={<BarChart3 />} className="debug-panel">
      {!token && (
        <div className="debug-warning">未配置 token，已暂停 debug 请求。请先在右上角设置 token。</div>
      )}
      {debugApiUnavailable && (
        <div className="debug-warning">Debug API unavailable: /api/debug/* 返回 Not Found。请确认 easyarm_web_bridge 已更新并重启。</div>
      )}
      {error && <div className="debug-error">{error}</div>}

      <div className="debug-grid">
        <section className="debug-section">
          <div className="debug-section-title">Logger</div>
          <div className="debug-status-grid">
            <Metric label="active" value={status?.active ? "true" : "false"} />
            <Metric label="file" value={status?.path ? status.path.split("/").pop() ?? status.path : "-"} />
            <Metric label="written" value={String(status?.written_count ?? 0)} />
            <Metric label="dropped" value={String(status?.dropped_count ?? 0)} />
          </div>
          <div className="debug-actions">
            <button disabled={!token || debugApiUnavailable || status?.active === true || Boolean(recordLoading)} onClick={() => void startRecording()}>
              <Play /> {recordLoading === "start" ? "Starting..." : "Start Recording"}
            </button>
            <button className="danger-button" disabled={!token || debugApiUnavailable || status?.active !== true || Boolean(recordLoading)} onClick={() => void stopRecording()}>
              <Square /> {recordLoading === "stop" ? "Stopping..." : "Stop Recording"}
            </button>
            <button className="ghost-button" disabled={!token || statusLoading} onClick={() => void refreshStatus()}>
              <RefreshCw /> Status
            </button>
          </div>
        </section>

        <section className="debug-section">
          <div className="debug-section-head">
            <div className="debug-section-title">Logs</div>
            <button className="ghost-button" disabled={!token || logsLoading} onClick={() => void refreshLogs()}>
              <RefreshCw /> {logsLoading ? "Refreshing..." : "Refresh"}
            </button>
          </div>
          {logs.length === 0 ? (
            <div className="empty-hint">没有 debug bin 日志</div>
          ) : (
            <div className="debug-log-list">
              {logs.map((log) => (
                <button
                  key={log.name}
                  className={`debug-log-row ${selectedLog === log.name ? "active" : ""}`}
                  onClick={() => {
                    setSelectedLog(log.name);
                    setPoints([]);
                  }}
                >
                  <span>{log.name}</span>
                  <small>{formatBytes(log.size)} · {formatTime(log.mtime)}</small>
                </button>
              ))}
            </div>
          )}
          <div className="debug-actions">
            <a className={`download-link ${selectedLog ? "" : "disabled"}`} href={downloadHref} download={selectedLog || undefined}>
              <Download /> Download bin
            </a>
          </div>
        </section>
      </div>

      <section className="debug-section debug-analysis">
        <div className="debug-section-title">Analysis</div>
        <div className="debug-form-grid">
          <label>
            <span>log</span>
            <input value={selectedLogEntry?.name ?? ""} readOnly placeholder="选择日志" />
          </label>
          <label>
            <span>joint</span>
            <select value={joint} onChange={(e) => setJoint(Number(e.target.value))}>
              {JOINTS.map((name, index) => <option key={name} value={index}>{name}</option>)}
            </select>
          </label>
          <label>
            <span>field</span>
            <select value={field} onChange={(e) => setField(e.target.value as DebugField)}>
              {DEBUG_FIELDS.map((item) => <option key={item} value={item}>{item}</option>)}
            </select>
          </label>
          <label>
            <span>start (s)</span>
            <input type="number" step="0.1" value={start} onChange={(e) => setStart(Number(e.target.value))} />
          </label>
          <label>
            <span>end (s)</span>
            <input type="number" step="0.1" value={end} onChange={(e) => setEnd(Number(e.target.value))} />
          </label>
          <label>
            <span>stride</span>
            <input type="number" min="1" step="1" value={stride} onChange={(e) => setStride(Math.max(1, Number(e.target.value)))} />
          </label>
          <button disabled={!token || debugApiUnavailable || !selectedLog || dataLoading} onClick={() => void loadData()}>
            <BarChart3 /> {dataLoading ? "Loading..." : "Load Data"}
          </button>
        </div>

        <DebugChart points={points} field={field} />
      </section>
    </Panel>
  );
}

function DebugChart({ points, field }: { points: DebugDataPoint[]; field: DebugField }) {
  if (points.length === 0) {
    return <div className="debug-chart empty-hint">加载数据后显示曲线</div>;
  }

  const width = 900;
  const height = 260;
  const pad = { left: 54, right: 18, top: 18, bottom: 34 };
  const xs = points.map((p) => p.time_s);
  const ys = points.map((p) => p.value);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const xSpan = maxX - minX || 1;
  const ySpan = maxY - minY || 1;
  const innerW = width - pad.left - pad.right;
  const innerH = height - pad.top - pad.bottom;
  const x = (value: number) => pad.left + ((value - minX) / xSpan) * innerW;
  const y = (value: number) => pad.top + innerH - ((value - minY) / ySpan) * innerH;
  const path = points.map((p, i) => `${i === 0 ? "M" : "L"}${x(p.time_s).toFixed(2)},${y(p.value).toFixed(2)}`).join(" ");

  return (
    <div className="debug-chart">
      <svg viewBox={`0 0 ${width} ${height}`} role="img" aria-label={`${field} chart`}>
        <rect x="0" y="0" width={width} height={height} rx="8" />
        <line x1={pad.left} y1={height - pad.bottom} x2={width - pad.right} y2={height - pad.bottom} />
        <line x1={pad.left} y1={pad.top} x2={pad.left} y2={height - pad.bottom} />
        <path d={path} />
        <text x={pad.left} y={height - 10}>time_s {numberText(minX, 2)} - {numberText(maxX, 2)}</text>
        <text x={pad.left} y="14">{field}: {numberText(minY, 4)} - {numberText(maxY, 4)}</text>
      </svg>
    </div>
  );
}

function formatBytes(value: number): string {
  if (value < 1024) return `${value} B`;
  if (value < 1024 * 1024) return `${(value / 1024).toFixed(1)} KiB`;
  return `${(value / (1024 * 1024)).toFixed(1)} MiB`;
}

function formatTime(value: number): string {
  if (!Number.isFinite(value)) return "n/a";
  return new Date(value * 1000).toLocaleString();
}
