import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { LineChart } from "echarts/charts";
import { DataZoomComponent, GridComponent, LegendComponent, TitleComponent, TooltipComponent } from "echarts/components";
import { init, use, type ECharts } from "echarts/core";
import { CanvasRenderer } from "echarts/renderers";
import { BarChart3, Download, Maximize2, Play, RefreshCw, Square, Trash2, Upload, X } from "lucide-react";
import { api, apiAssetUrl, ApiError, errorMessage } from "../api/client";
import type { DebugDataPoint, DebugField, DebugLogEntry, DebugStatusResponse } from "../api/types";

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

const FIELD_COLORS: Record<DebugField, string> = {
  position_error: "#dc2626",
  smoothed_position_error: "#f97316",
  velocity_error: "#ca8a04",
  motor_velocity_error: "#65a30d",
  command_position: "#059669",
  state_position: "#0d9488",
  command_velocity: "#0284c7",
  state_velocity: "#2563eb",
  motor_torque: "#7c3aed",
  write_duration_us: "#be185d",
};

const JOINTS = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"];

use([LineChart, GridComponent, TooltipComponent, LegendComponent, DataZoomComponent, TitleComponent, CanvasRenderer]);

type DebugDataPanelProps = {
  token: string;
};

type FieldSeries = {
  field: DebugField;
  points: DebugDataPoint[];
};

type JointSeries = {
  joint: number;
  series: FieldSeries[];
};

type AnalysisViewProps = {
  token: string;
  debugApiUnavailable: boolean;
  selectedLogEntry: DebugLogEntry | null;
  selectedJoints: number[];
  selectedFields: DebugField[];
  start: number;
  end: number;
  stride: number;
  dataLoading: boolean;
  chartData: JointSeries[];
  setSelectedJoints: (value: number[]) => void;
  setSelectedFields: (value: DebugField[]) => void;
  setStart: (value: number) => void;
  setEnd: (value: number) => void;
  setStride: (value: number) => void;
  loadData: () => void;
  fullscreen?: boolean;
  onOpenFullscreen?: () => void;
  onCloseFullscreen?: () => void;
};

export function DebugDataPanel({ token }: DebugDataPanelProps) {
  const [status, setStatus] = useState<DebugStatusResponse | null>(null);
  const [logs, setLogs] = useState<DebugLogEntry[]>([]);
  const [selectedLog, setSelectedLog] = useState("");
  const [selectedJoints, setSelectedJoints] = useState<number[]>([0]);
  const [selectedFields, setSelectedFields] = useState<DebugField[]>(["position_error"]);
  const [start, setStart] = useState(0);
  const [end, setEnd] = useState(10);
  const [stride, setStride] = useState(5);
  const [chartData, setChartData] = useState<JointSeries[]>([]);
  const [statusLoading, setStatusLoading] = useState(false);
  const [recordLoading, setRecordLoading] = useState<"start" | "stop" | "">("");
  const [logsLoading, setLogsLoading] = useState(false);
  const [dataLoading, setDataLoading] = useState(false);
  const [uploadLoading, setUploadLoading] = useState(false);
  const [deletingLog, setDeletingLog] = useState("");
  const [fullscreen, setFullscreen] = useState(false);
  const [open, setOpen] = useState(false);
  const [error, setError] = useState("");
  const [debugApiUnavailable, setDebugApiUnavailable] = useState(false);
  const uploadInputRef = useRef<HTMLInputElement>(null);
  const loggerSectionRef = useRef<HTMLElement>(null);
  const [loggerSectionHeight, setLoggerSectionHeight] = useState<number | null>(null);

  const selectedLogEntry = useMemo(() => logs.find((item) => item.name === selectedLog) ?? null, [logs, selectedLog]);
  const activeLogName = status?.path ? status.path.split("/").pop() ?? status.path : "-";

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

  const refreshStatus = useCallback(async (showLoading = true) => {
    if (!token) return;
    if (showLoading) setStatusLoading(true);
    try {
      setStatus(await api.debugStatus);
      setDebugApiUnavailable(false);
      setError("");
    } catch (err) {
      handleRequestError(err);
    } finally {
      if (showLoading) setStatusLoading(false);
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
        setChartData([]);
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
      void refreshStatus(false);
    }, 1500);
    return () => window.clearInterval(timer);
  }, [debugApiUnavailable, refreshStatus, token]);

  useEffect(() => {
    const element = loggerSectionRef.current;
    if (!element) return;

    const updateHeight = () => {
      const nextHeight = Math.ceil(element.getBoundingClientRect().height);
      setLoggerSectionHeight((current) => current === nextHeight ? current : nextHeight);
    };

    updateHeight();
    const observer = new ResizeObserver(updateHeight);
    observer.observe(element);
    window.addEventListener("resize", updateHeight);
    return () => {
      observer.disconnect();
      window.removeEventListener("resize", updateHeight);
    };
  }, []);

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

  const uploadBin = async (file: File | undefined) => {
    if (!token || debugApiUnavailable || !file) return;
    setUploadLoading(true);
    setError("");
    try {
      const response = await api.debugUpload(file);
      const logsResponse = await api.debugLogs;
      setLogs(logsResponse.logs);
      setSelectedLog(response.log.name);
      setChartData([]);
      setDebugApiUnavailable(false);
    } catch (err) {
      handleRequestError(err);
    } finally {
      setUploadLoading(false);
      if (uploadInputRef.current) {
        uploadInputRef.current.value = "";
      }
    }
  };

  const deleteLog = async (name: string) => {
    if (!token || debugApiUnavailable || deletingLog) return;
    setDeletingLog(name);
    setError("");
    try {
      await api.debugDelete(name);
      setLogs((current) => current.filter((log) => log.name !== name));
      if (selectedLog === name) {
        setSelectedLog("");
        setChartData([]);
      }
      setDebugApiUnavailable(false);
    } catch (err) {
      handleRequestError(err);
    } finally {
      setDeletingLog("");
    }
  };

  const loadData = async () => {
    if (!token || debugApiUnavailable || !selectedLog || selectedJoints.length === 0 || selectedFields.length === 0) return;
    setDataLoading(true);
    setError("");
    try {
      const requests = selectedJoints.flatMap((joint) => selectedFields.map((field) => ({ joint, field })));
      const results = await Promise.allSettled(
        requests.map((item) => api.debugData({ name: selectedLog, joint: item.joint, field: item.field, start, end, stride })),
      );

      const fulfilled: JointSeries[] = [];
      const failures: unknown[] = [];
      for (let i = 0; i < results.length; i++) {
        const result = results[i];
        const request = requests[i];
        if (result.status === "fulfilled") {
          const existing = fulfilled.find((item) => item.joint === request.joint);
          const series = { field: request.field, points: result.value.points };
          if (existing) {
            existing.series.push(series);
          } else {
            fulfilled.push({ joint: request.joint, series: [series] });
          }
        } else {
          failures.push(result.reason);
        }
      }

      if (fulfilled.length === 0) {
        setChartData([]);
        handleRequestError(failures[0] ?? new Error("no debug data returned"));
        return;
      }

      setChartData(fulfilled.sort((a, b) => a.joint - b.joint));
      if (failures.length > 0) {
        setError(`部分曲线加载失败：${failures.length}/${results.length}`);
      }
    } finally {
      setDataLoading(false);
    }
  };

  const downloadHref = selectedLog ? apiAssetUrl(`/api/debug/logs/${encodeURIComponent(selectedLog)}/download`) : "";

  return (
    <details className="panel stream-section debug-panel" open={open}>
      <summary
        onClick={(event) => {
          event.preventDefault();
          setOpen((value) => !value);
        }}
      >
        <span><BarChart3 /> Debug / Data Analysis</span>
        <small>Logger / Logs / Analysis</small>
      </summary>
      <div className="debug-panel-body">
        {!token && (
          <div className="debug-warning">未配置 token，已暂停 debug 请求。请先在右上角设置 token。</div>
        )}
        {debugApiUnavailable && (
          <div className="debug-warning">Debug API unavailable: /api/debug/* 返回 Not Found。请确认 easyarm_web_bridge 已更新并重启。</div>
        )}
        {error && <div className="debug-error">{error}</div>}

        <div className="debug-grid">
          <section ref={loggerSectionRef} className="debug-section logger-section">
            <div className="debug-section-title">Logger</div>
            <div className="logger-summary">
              <div className="logger-metric">
                <span>active</span>
                <strong>{status?.active ? "true" : "false"}</strong>
              </div>
              <div className="logger-metric">
                <span>written</span>
                <strong>{status?.written_count ?? 0}</strong>
              </div>
              <div className="logger-metric">
                <span>dropped</span>
                <strong>{status?.dropped_count ?? 0}</strong>
              </div>
              <div className="logger-file">
                <span>file</span>
                <strong title={activeLogName}>{activeLogName}</strong>
              </div>
            </div>
            <div className="debug-actions logger-actions">
              <button disabled={!token || debugApiUnavailable || status?.active === true || Boolean(recordLoading)} onClick={() => void startRecording()}>
                <Play /> {recordLoading === "start" ? "Starting..." : "Start"}
              </button>
              <button className="danger-button" disabled={!token || debugApiUnavailable || status?.active !== true || Boolean(recordLoading)} onClick={() => void stopRecording()}>
                <Square /> {recordLoading === "stop" ? "Stopping..." : "Stop"}
              </button>
              <button className="ghost-button" disabled={!token || statusLoading} onClick={() => void refreshStatus()}>
                <RefreshCw /> {statusLoading ? "Refreshing..." : "Status"}
              </button>
            </div>
          </section>

          <section className="debug-section logs-section" style={loggerSectionHeight ? { height: loggerSectionHeight } : undefined}>
            <div className="debug-section-head">
              <div className="debug-section-title">Logs</div>
            </div>
            {logs.length === 0 ? (
              <div className="empty-hint">没有 debug bin 日志</div>
            ) : (
              <div className="debug-log-list">
                {logs.map((log) => (
                  <div key={log.name} className={`debug-log-row ${selectedLog === log.name ? "active" : ""}`}>
                    <button
                      className="debug-log-select"
                      onClick={() => {
                        setSelectedLog(log.name);
                        setChartData([]);
                      }}
                    >
                      <span>{log.name}</span>
                      <small>{formatBytes(log.size)} · {formatTime(log.mtime)}</small>
                    </button>
                    <button
                      className="debug-log-delete"
                      disabled={!token || debugApiUnavailable || Boolean(deletingLog)}
                      title={`Delete ${log.name}`}
                      aria-label={`Delete ${log.name}`}
                      onClick={() => void deleteLog(log.name)}
                    >
                      <Trash2 />
                    </button>
                  </div>
                ))}
              </div>
            )}
            <div className="debug-actions logs-actions">
              <a className={`download-link ${selectedLog ? "" : "disabled"}`} href={downloadHref} download={selectedLog || undefined}>
                <Download /> Download bin
              </a>
              <button className="ghost-button upload-button" disabled={!token || debugApiUnavailable || uploadLoading} onClick={() => uploadInputRef.current?.click()}>
                <Upload /> {uploadLoading ? "Uploading..." : "Upload bin"}
              </button>
              <button className="ghost-button refresh-button" disabled={!token || logsLoading} onClick={() => void refreshLogs()}>
                <RefreshCw /> {logsLoading ? "Refreshing..." : "Refresh"}
              </button>
              <input
                ref={uploadInputRef}
                className="hidden-file-input"
                type="file"
                accept=".bin,application/octet-stream"
                onChange={(event) => void uploadBin(event.currentTarget.files?.[0])}
              />
            </div>
          </section>
        </div>

        <AnalysisView
          token={token}
          debugApiUnavailable={debugApiUnavailable}
          selectedLogEntry={selectedLogEntry}
          selectedJoints={selectedJoints}
          selectedFields={selectedFields}
          start={start}
          end={end}
          stride={stride}
          dataLoading={dataLoading}
          chartData={chartData}
          setSelectedJoints={setSelectedJoints}
          setSelectedFields={setSelectedFields}
          setStart={setStart}
          setEnd={setEnd}
          setStride={setStride}
          loadData={loadData}
          onOpenFullscreen={() => setFullscreen(true)}
        />

        {fullscreen && (
          <div className="debug-fullscreen-backdrop">
            <div className="debug-fullscreen-panel">
              <AnalysisView
                token={token}
                debugApiUnavailable={debugApiUnavailable}
                selectedLogEntry={selectedLogEntry}
                selectedJoints={selectedJoints}
                selectedFields={selectedFields}
                start={start}
                end={end}
                stride={stride}
                dataLoading={dataLoading}
                chartData={chartData}
                setSelectedJoints={setSelectedJoints}
                setSelectedFields={setSelectedFields}
                setStart={setStart}
                setEnd={setEnd}
                setStride={setStride}
                loadData={loadData}
                fullscreen
                onCloseFullscreen={() => setFullscreen(false)}
              />
            </div>
          </div>
        )}
      </div>
    </details>
  );
}

function AnalysisView({
  token,
  debugApiUnavailable,
  selectedLogEntry,
  selectedJoints,
  selectedFields,
  start,
  end,
  stride,
  dataLoading,
  chartData,
  setSelectedJoints,
  setSelectedFields,
  setStart,
  setEnd,
  setStride,
  loadData,
  fullscreen = false,
  onOpenFullscreen,
  onCloseFullscreen,
}: AnalysisViewProps) {
  return (
    <section className={`debug-section debug-analysis ${fullscreen ? "fullscreen" : ""}`}>
      <div className="debug-section-head">
        <div className="debug-section-title">Analysis</div>
        {fullscreen ? (
          <button className="ghost-button" onClick={onCloseFullscreen}><X /> Close</button>
        ) : (
          <button className="ghost-button fullscreen-button" onClick={onOpenFullscreen} aria-label="Fullscreen">
            <Maximize2 />
            <span>Fullscreen</span>
          </button>
        )}
      </div>
      <div className="debug-form-grid multi">
        <label>
          <span>log</span>
          <input value={selectedLogEntry?.name ?? ""} readOnly placeholder="选择日志" />
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
        <button disabled={!token || debugApiUnavailable || !selectedLogEntry || dataLoading || selectedJoints.length === 0 || selectedFields.length === 0} onClick={() => void loadData()}>
          <BarChart3 /> {dataLoading ? "Loading..." : "Load Data"}
        </button>
      </div>

      <div className="debug-selector-grid">
        <MultiToggleGroup
          label="joints"
          options={JOINTS.map((name, index) => ({ value: index, label: name }))}
          selected={selectedJoints}
          onChange={setSelectedJoints}
        />
        <MultiToggleGroup
          label="fields"
          options={DEBUG_FIELDS.map((field) => ({ value: field, label: field }))}
          selected={selectedFields}
          onChange={setSelectedFields}
        />
      </div>

      <DebugCharts chartData={chartData} selectedFields={selectedFields} fullscreen={fullscreen} />
    </section>
  );
}

function MultiToggleGroup<T extends string | number>({
  label,
  options,
  selected,
  onChange,
}: {
  label: string;
  options: Array<{ value: T; label: string }>;
  selected: T[];
  onChange: (value: T[]) => void;
}) {
  const toggle = (value: T) => {
    if (selected.includes(value)) {
      onChange(selected.filter((item) => item !== value));
    } else {
      onChange([...selected, value]);
    }
  };

  return (
    <div className="debug-toggle-group">
      <span>{label}</span>
      <div>
        {options.map((option) => (
          <button
            key={String(option.value)}
            type="button"
            className={selected.includes(option.value) ? "active" : ""}
            onClick={() => toggle(option.value)}
          >
            {option.label}
          </button>
        ))}
      </div>
    </div>
  );
}

function DebugCharts({ chartData, selectedFields, fullscreen }: { chartData: JointSeries[]; selectedFields: DebugField[]; fullscreen: boolean }) {
  if (chartData.length === 0) {
    return <div className="debug-chart empty-hint">加载数据后显示曲线</div>;
  }

  return (
    <div className={`debug-chart-stack ${fullscreen ? "fullscreen" : ""}`}>
      {chartData.map((item) => (
        <JointChart key={item.joint} joint={item.joint} series={item.series} fullscreen={fullscreen} />
      ))}
    </div>
  );
}

function JointChart({ joint, series, fullscreen }: { joint: number; series: FieldSeries[]; fullscreen: boolean }) {
  const chartRef = useRef<HTMLDivElement>(null);
  const instanceRef = useRef<ECharts | null>(null);
  const allPoints = series.flatMap((item) => item.points);

  useEffect(() => {
    const element = chartRef.current;
    if (!element) return;
    const chart = init(element, undefined, { renderer: "canvas" });
    instanceRef.current = chart;
    const resizeObserver = new ResizeObserver(() => chart.resize());
    resizeObserver.observe(element);
    return () => {
      resizeObserver.disconnect();
      chart.dispose();
      instanceRef.current = null;
    };
  }, []);

  useEffect(() => {
    const chart = instanceRef.current;
    if (!chart) return;
    chart.setOption({
      animation: false,
      color: series.map((item) => FIELD_COLORS[item.field]),
      title: {
        text: JOINTS[joint],
        left: 12,
        top: 6,
        textStyle: { fontSize: 13, fontWeight: 800, color: "#1e332c" },
      },
      tooltip: {
        trigger: "axis",
        axisPointer: { type: "cross" },
        valueFormatter: (value: unknown) => typeof value === "number" ? value.toFixed(6) : String(value),
      },
      legend: {
        top: 6,
        right: 12,
        type: "scroll",
        textStyle: { color: "#425850", fontSize: 11, fontWeight: 700 },
      },
      grid: { left: 58, right: 22, top: 50, bottom: 48 },
      dataZoom: [
        { type: "inside", xAxisIndex: 0, filterMode: "none" },
        { type: "slider", xAxisIndex: 0, height: 20, bottom: 12, filterMode: "none" },
      ],
      xAxis: {
        type: "value",
        name: "time_s",
        nameLocation: "middle",
        nameGap: 28,
        axisLine: { lineStyle: { color: "#cbd8d2" } },
        axisLabel: { color: "#60736c" },
        splitLine: { lineStyle: { color: "#edf1ef" } },
      },
      yAxis: {
        type: "value",
        name: "value",
        axisLine: { lineStyle: { color: "#cbd8d2" } },
        axisLabel: { color: "#60736c" },
        splitLine: { lineStyle: { color: "#edf1ef" } },
      },
      series: series.map((item) => ({
        name: item.field,
        type: "line",
        showSymbol: false,
        lineStyle: { width: 2, color: FIELD_COLORS[item.field] },
        emphasis: { focus: "series" },
        data: item.points.map((point) => [point.time_s, point.value]),
      })),
    }, true);
    window.setTimeout(() => chart.resize(), 0);
  }, [fullscreen, joint, series]);

  if (allPoints.length === 0) {
    return <div className="debug-chart empty-hint">{JOINTS[joint]} 没有数据</div>;
  }

  return (
    <div className="debug-chart">
      <div className={`debug-echart ${fullscreen ? "fullscreen" : ""}`} ref={chartRef} />
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
