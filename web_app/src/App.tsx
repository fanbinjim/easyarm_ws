import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  Activity,
  ShieldCheck,
  Loader2,
  Gauge,
} from "lucide-react";

import { api, apiPost, errorMessage, isRecoverableError, recoverableMessage } from "./api/client";
import { useSettings } from "./hooks/useSettings";
import { useHealth } from "./hooks/useHealth";
import { useApiState } from "./hooks/useApiState";
import { useTelemetry } from "./hooks/useTelemetry";

import { ToastProvider, useToast } from "./components/Toast";
import { ConfirmDialog, type ConfirmDialogState } from "./components/ConfirmDialog";
import { SettingsDialog } from "./components/SettingsDialog";
import { StatusBar } from "./components/StatusBar";
import { RobotViewer } from "./components/RobotViewer";
import { MotionPanel } from "./components/MotionPanel";
import { StreamPanel } from "./components/StreamPanel";
import { JointTable } from "./components/JointTable";
import { PosePanel } from "./components/PosePanel";
import { ActionLog } from "./components/ActionLog";
import { ControllerList } from "./components/ControllerList";
import { RosLog } from "./components/RosLog";
import { SummaryCard } from "./ui/SummaryCard";
import type { JointTarget } from "./api/types";
import type { PoseValues } from "./ui/PoseEditor";

function stepDigits(step: number): number {
  const text = String(step);
  if (!text.includes(".")) return 0;
  return text.split(".")[1]?.length ?? 0;
}

function setNativeInputValue(input: HTMLInputElement, value: string): void {
  const valueSetter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")?.set;
  valueSetter?.call(input, value);
  input.dispatchEvent(new Event("input", { bubbles: true }));
}

function AppInner() {
  const settings = useSettings();
  const health = useHealth(settings.token);
  const apiState = useApiState(settings.token, health.data);
  const telemetry = useTelemetry(settings.token);
  const toast = useToast();

  useEffect(() => {
    const handleNumberWheel = (event: WheelEvent) => {
      const target = event.target;
      if (!(target instanceof HTMLInputElement) || target.type !== "number") return;
      if (document.activeElement !== target) return;

      event.preventDefault();
      event.stopPropagation();

      const step = target.step && target.step !== "any" ? Number(target.step) : 1;
      const delta = event.deltaY < 0 ? step : -step;
      const current = target.value === "" ? 0 : Number(target.value);
      if (!Number.isFinite(step) || !Number.isFinite(current)) return;

      const min = target.min === "" ? -Infinity : Number(target.min);
      const max = target.max === "" ? Infinity : Number(target.max);
      const next = Math.min(max, Math.max(min, current + delta));
      setNativeInputValue(target, next.toFixed(stepDigits(step)));
    };

    document.addEventListener("wheel", handleNumberWheel, { capture: true, passive: false });
    return () => document.removeEventListener("wheel", handleNumberWheel, { capture: true });
  }, []);

  const [planOnly, setPlanOnly] = useState(true);
  const [velocityScale, setVelocityScale] = useState(0.1);
  const [accelScale, setAccelScale] = useState(0.1);
  const [jointTarget, setJointTarget] = useState<JointTarget | null>(null);
  const [moveLTarget, setMoveLTarget] = useState<PoseValues | null>(null);
  const [confirm, setConfirm] = useState<ConfirmDialogState | null>(null);
  const busyRef = useRef(false);
  const shutdownReqRef = useRef(false);

  const activeAction = telemetry.data?.active_action;
  const activeActionInFlight = Boolean(activeAction?.done === false && activeAction?.kind);
  const jointAge = telemetry.data?.latest_joint_age_sec;
  const jointStale = jointAge != null && jointAge > 2;
  const jointAgeText = jointAge == null ? "n/a" : jointStale ? `stale ${jointAge.toFixed(1)}s` : `${jointAge.toFixed(2)}s`;
  const serviceStopped = shutdownReqRef.current || (health.backendStatus === "connected" && health.motionServerStatus === "unavailable");
  const allCoreReady = Boolean(
    health.data?.controller_manager &&
    health.data?.joint_state_recent &&
    health.data?.motion_server?.movej &&
    health.data?.motion_server?.movel,
  );

  const healthSummaryValue = health.backendStatus === "unauthorized" ? "Auth"
    : health.backendStatus !== "connected" ? "Bridge Off"
    : health.motionServerStatus === "unavailable" ? "Motion Off"
    : allCoreReady && !jointStale && !health.warning ? "Ready"
    : "Degraded";

  const healthSummaryDetail = health.backendStatus === "unauthorized" ? health.fatalError
    : health.backendStatus !== "connected" ? "Web bridge 未连接"
    : health.motionServerStatus === "unavailable" ? "Motion server 不可用"
    : health.warning ? health.warning
    : health.data?.joint_state_recent === false ? "关节状态未更新"
    : `joint age ${jointAgeText}`;

  const healthSummaryTone: "good" | "warn" | "bad" = health.backendStatus === "unauthorized" || health.backendStatus === "error" || serviceStopped ? "bad"
    : allCoreReady && !jointStale && !health.warning ? "good"
    : "warn";

  const openConfirm = useCallback((dialog: ConfirmDialogState) => setConfirm(dialog), []);
  const closeConfirm = useCallback(() => setConfirm(null), []);

  const post = useCallback(async <T,>(path: string, payload?: unknown): Promise<T> => {
    busyRef.current = true;
    try {
      const res = await apiPost<T>(path, payload);
      busyRef.current = false;
      return res;
    } catch (err) {
      busyRef.current = false;
      if (isRecoverableError(err)) {
        toast.toast("warning", "请求处理中", recoverableMessage(err) ?? undefined);
      } else {
        toast.toast("error", errorMessage(err));
      }
      throw err;
    }
  }, [toast]);

  const handleMoveJ = useCallback((joints: number[]) => {
    post("/api/movej", { joints, velocity_scale: velocityScale, acceleration_scale: accelScale, execute: !planOnly }).catch(() => undefined);
  }, [post, velocityScale, accelScale, planOnly]);

  const handleMoveL = useCallback((pose: PoseValues) => {
    post("/api/movel", { ...pose, frame_id: "base_link", velocity_scale: velocityScale, acceleration_scale: accelScale, execute: !planOnly }).catch(() => undefined);
  }, [post, velocityScale, accelScale, planOnly]);

  const handleMoveNamed = useCallback((name: string) => {
    post("/api/move-named-state", { name, velocity_scale: velocityScale, acceleration_scale: accelScale, execute: !planOnly }).catch(() => undefined);
  }, [post, velocityScale, accelScale, planOnly]);

  const handleCancelAction = useCallback(() => {
    post("/api/actions/active/cancel").catch(() => undefined);
  }, [post]);

  const handleStop = useCallback(() => {
    post("/api/stop").catch(() => undefined);
  }, [post]);

  const handlePlanOnlyChange = useCallback((v: boolean) => {
    if (v) { setPlanOnly(true); return; }
    openConfirm({
      title: "切换到执行模式",
      message: "执行模式下 MoveJ、MoveL 和 Named State 会下发真实运动。",
      confirmLabel: "切换到执行",
      tone: "danger",
      onConfirm: () => setPlanOnly(false),
    });
  }, [openConfirm]);

  const handleSafeShutdown = useCallback(() => {
    openConfirm({
      title: "安全关机",
      message: "确认执行安全关机？这会触发机械臂安全停机流程：停止当前运动 → 移动到 ready 位 → 停止 controller → 禁用 hardware → 关闭 bringup。",
      confirmLabel: "执行安全关机",
      tone: "danger",
      onConfirm: () => {
        shutdownReqRef.current = true;
        post("/api/safe-shutdown").finally(() => { shutdownReqRef.current = false; });
      },
    });
  }, [openConfirm, post]);

  const jointNames = useMemo(() => {
    return apiState.joints?.names ?? health.data
      ? ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
      : [];
  }, [apiState.joints?.names, health.data]);

  const handleCopyPoseToServoL = useCallback(() => {
    /* Handled by StreamPanel via local state; this is a placeholder */
  }, []);

  return (
    <div className={`app-shell ${planOnly ? "plan-mode" : "execute-mode"}`}>
      <StatusBar
        backendStatus={health.backendStatus}
        motionServerStatus={health.motionServerStatus}
        telemetryConnected={telemetry.connected}
        telemetryFreshness={telemetry.freshness}
        telemetry={telemetry.data}
        stateMode={apiState.state?.mode ?? "UNKNOWN"}
        isMockHardware={health.isMockHardware}
        activeActionInFlight={activeActionInFlight}
        onCancel={handleCancelAction}
        onStop={handleStop}
        onSafeShutdown={handleSafeShutdown}
        onOpenSettings={settings.openDialog}
        busy={busyRef.current}
      />

      {health.fatalError && (
        <div className={health.backendStatus === "unauthorized" ? "auth-text" : "service-warning"}>
          {health.fatalError}
        </div>
      )}

      {(health.backendStatus === "connected" && health.motionServerStatus === "unavailable" && !health.fatalError) && (
        <div className="service-warning">Motion server 不可用，运动命令暂时无法执行。</div>
      )}

      {(health.backendStatus === "connected" && health.data && !health.data.controller_manager && !health.fatalError) && (
        <div className="service-warning">Controller manager 不可用，控制器状态无法读取。</div>
      )}

      <section className="summary-grid">
        <SummaryCard
          icon={<Activity />}
          label="系统"
          value={apiState.state?.busy ? "Busy" : "Idle"}
          detail={`task: ${apiState.state?.active_task || "idle"}`}
          tone={apiState.state?.busy ? "warn" : "good"}
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
          <RobotViewer token={settings.token} telemetry={telemetry.data} jointTarget={jointTarget} moveLTarget={moveLTarget} />
          <MotionPanel
            namedStates={apiState.namedStates}
            planOnly={planOnly}
            velocityScale={velocityScale}
            accelScale={accelScale}
            onVelocityChange={setVelocityScale}
            onAccelChange={setAccelScale}
            onPlanOnlyChange={handlePlanOnlyChange}
            onMoveJ={handleMoveJ}
            onJointTargetChange={setJointTarget}
            onMoveL={handleMoveL}
            onMoveLTargetChange={setMoveLTarget}
            onMoveNamedState={handleMoveNamed}
            busy={busyRef.current}
          />
          <StreamPanel />
        </div>

        <aside className="workspace-side">
          <JointTable joints={apiState.joints} jointNames={jointNames} />
          <PosePanel pose={apiState.pose} onCopyToServoL={handleCopyPoseToServoL} />
          <ActionLog telemetry={telemetry.data} onCancel={handleCancelAction} />
          <ControllerList controllers={apiState.controllers} />
        </aside>
      </section>

      <RosLog telemetry={telemetry.data} />

      {confirm && <ConfirmDialog dialog={confirm} onClose={closeConfirm} />}
      {settings.open && (
        <SettingsDialog
          draftToken={settings.draftToken}
          setDraftToken={settings.setDraftToken}
          draftUrl={settings.draftUrl}
          setDraftUrl={settings.setDraftUrl}
          onSave={settings.save}
          onCancel={settings.cancel}
        />
      )}
    </div>
  );
}

export default function App() {
  return (
    <ToastProvider>
      <AppInner />
    </ToastProvider>
  );
}
