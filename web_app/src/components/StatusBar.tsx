import { CircleStop, Power, Settings, Wifi, WifiOff, Gauge, Activity } from "lucide-react";
import { StatusPill } from "../ui/StatusPill";
import type { Telemetry, BackendStatus, MotionServerStatus, TelemetryFreshness } from "../api/types";

type Props = {
  backendStatus: BackendStatus;
  motionServerStatus: MotionServerStatus;
  telemetryConnected: boolean;
  telemetryFreshness: TelemetryFreshness;
  telemetry: Telemetry | null;
  stateMode: string;
  isMockHardware: "true" | "false" | "unknown";
  activeActionInFlight: boolean;
  onCancel: () => void;
  onStop: () => void;
  onSafeShutdown: () => void;
  onOpenSettings: () => void;
  busy: boolean;
};

const modeLabel = (mode: string) => {
  if (!mode || mode === "UNKNOWN") return "UNKNOWN";
  return mode;
};

export function StatusBar({
  backendStatus,
  motionServerStatus,
  telemetryConnected,
  telemetryFreshness,
  telemetry,
  stateMode,
  isMockHardware,
  activeActionInFlight,
  onCancel,
  onStop,
  onSafeShutdown,
  onOpenSettings,
  busy,
}: Props) {
  const bridgeLabel = backendStatus === "unauthorized" ? "Auth Required"
    : backendStatus === "connected" ? "Bridge OK"
    : backendStatus === "error" ? "Bridge Error"
    : "Bridge Off";
  const bridgeTone = backendStatus === "connected" ? "good" : "bad";

  const telemetryLabel = !telemetryConnected ? "Telemetry Off"
    : telemetryFreshness === "stale" ? "Telemetry Stale"
    : "Telemetry";
  const telemetryTone = telemetryConnected && telemetryFreshness === "fresh" ? "good" : "warn";

  const modeTone = stateMode === "POSITION" ? "good" : "warn";

  const cancelLabel = activeActionInFlight
    ? `取消 ${telemetry?.active_action?.kind ?? ""}`
    : "Cancel";

  return (
    <header className="status-bar">
      <div className="status-bar-left">
        <div className="app-brand">
          <span className="eyebrow">EasyArm A1</span>
          <h1 className="app-title">控制台</h1>
        </div>
        <StatusPill label={bridgeLabel} tone={bridgeTone} icon={backendStatus === "connected" ? <Wifi /> : <WifiOff />} />
        <StatusPill label={telemetryLabel} tone={telemetryTone} icon={telemetryConnected ? <Wifi /> : <WifiOff />} />
        <StatusPill label={modeLabel(stateMode)} tone={modeTone} icon={<Gauge />} />
        {isMockHardware === "true" && (
          <StatusPill label="MOCK" tone="bad" icon={<Activity />} />
        )}
        {motionServerStatus === "unavailable" && (
          <StatusPill label="Motion Off" tone="bad" icon={<Activity />} />
        )}
        {motionServerStatus === "degraded" && (
          <StatusPill label="Motion Degraded" tone="warn" icon={<Activity />} />
        )}
      </div>
      <div className="status-bar-right">
        <button
          className={activeActionInFlight ? "danger-button" : "danger-button"}
          onClick={activeActionInFlight ? onCancel : onStop}
          disabled={busy}
        >
          <CircleStop /> {cancelLabel}
        </button>
        <button className="shutdown-button" onClick={onSafeShutdown} disabled={busy}>
          <Power /> 安全关机
        </button>
        <button className="settings-button" onClick={onOpenSettings}>
          <Settings />
        </button>
      </div>
    </header>
  );
}
