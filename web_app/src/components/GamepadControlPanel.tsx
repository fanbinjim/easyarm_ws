import { useCallback, useEffect, useRef, useState } from "react";
import { Gamepad2, Pause, RefreshCw } from "lucide-react";
import type { PoseResponse, StreamCommand } from "../api/types";
import type { PoseValues } from "../ui/PoseEditor";

type GamepadMode = "speedl" | "servol";

type Props = {
  pose: PoseResponse | null;
  onSend: (command: StreamCommand) => void;
  onHalt: () => void;
};

const DEADZONE = 0.12;
const SEND_HZ = 40;
const BASE_LINEAR_SPEED = 0.05;
const BASE_ANGULAR_SPEED = 0.45;
const SLOW_SCALE = 0.35;
const FAST_SCALE = 3.0;
const BUTTON_A = 0;
const BUTTON_B = 1;
const BUTTON_X = 2;
const BUTTON_Y = 3;
const BUTTON_LB = 4;
const BUTTON_RB = 5;
const BUTTON_BACK = 8;
const BUTTON_START = 9;
const DPAD_UP = 12;
const DPAD_DOWN = 13;

function applyDeadzone(value: number): number {
  if (Math.abs(value) < DEADZONE) return 0;
  const normalized = (Math.abs(value) - DEADZONE) / (1 - DEADZONE);
  return Math.sign(value) * normalized * normalized;
}

function pressed(gamepad: Gamepad | null, index: number): boolean {
  return Boolean(gamepad?.buttons[index]?.pressed);
}

function poseToValues(pose: PoseResponse | null): PoseValues | null {
  if (!pose) return null;
  return {
    x: pose.position.x,
    y: pose.position.y,
    z: pose.position.z,
    qx: pose.orientation.x,
    qy: pose.orientation.y,
    qz: pose.orientation.z,
    qw: pose.orientation.w,
  };
}

function normalizeQuaternion(pose: PoseValues): PoseValues {
  const norm = Math.hypot(pose.qx, pose.qy, pose.qz, pose.qw) || 1;
  return {
    ...pose,
    qx: pose.qx / norm,
    qy: pose.qy / norm,
    qz: pose.qz / norm,
    qw: pose.qw / norm,
  };
}

function multiplyQuaternion(
  a: Pick<PoseValues, "qx" | "qy" | "qz" | "qw">,
  b: Pick<PoseValues, "qx" | "qy" | "qz" | "qw">,
) {
  return {
    qx: a.qw * b.qx + a.qx * b.qw + a.qy * b.qz - a.qz * b.qy,
    qy: a.qw * b.qy - a.qx * b.qz + a.qy * b.qw + a.qz * b.qx,
    qz: a.qw * b.qz + a.qx * b.qy - a.qy * b.qx + a.qz * b.qw,
    qw: a.qw * b.qw - a.qx * b.qx - a.qy * b.qy - a.qz * b.qz,
  };
}

function deltaQuaternion(wx: number, wy: number, wz: number, dt: number) {
  const angle = Math.hypot(wx, wy, wz) * dt;
  if (angle < 1e-9) return { qx: 0, qy: 0, qz: 0, qw: 1 };
  const half = angle / 2;
  const scale = Math.sin(half) / Math.hypot(wx, wy, wz);
  return {
    qx: wx * scale,
    qy: wy * scale,
    qz: wz * scale,
    qw: Math.cos(half),
  };
}

function buildTwist(gamepad: Gamepad, scale: number): number[] {
  const axes = gamepad.axes;
  const vx = applyDeadzone(axes[0] ?? 0) * BASE_LINEAR_SPEED * scale;
  const vy = -applyDeadzone(axes[1] ?? 0) * BASE_LINEAR_SPEED * scale;
  const vz = (pressed(gamepad, DPAD_UP) ? 1 : 0) + (pressed(gamepad, DPAD_DOWN) ? -1 : 0);
  const wx = applyDeadzone(axes[2] ?? 0) * BASE_ANGULAR_SPEED * scale;
  const wy = -applyDeadzone(axes[3] ?? 0) * BASE_ANGULAR_SPEED * scale;
  const wz = (pressed(gamepad, BUTTON_X) ? 1 : 0) + (pressed(gamepad, BUTTON_B) ? -1 : 0);
  return [
    vx,
    vy,
    vz * BASE_LINEAR_SPEED * scale,
    wx,
    wy,
    wz * BASE_ANGULAR_SPEED * scale,
  ];
}

export function GamepadControlPanel({ pose, onSend, onHalt }: Props) {
  const [enabled, setEnabled] = useState(false);
  const [mode, setMode] = useState<GamepadMode>("speedl");
  const [status, setStatus] = useState("未启用");
  const [gamepadName, setGamepadName] = useState("");
  const [scaleLabel, setScaleLabel] = useState("1.00x");
  const targetPoseRef = useRef<PoseValues | null>(null);
  const latestPoseRef = useRef<PoseResponse | null>(pose);
  const lastTimeRef = useRef(0);
  const previousButtonsRef = useRef<boolean[]>([]);
  const haltedRef = useRef(true);
  const statusRef = useRef(status);
  const gamepadNameRef = useRef(gamepadName);
  const scaleLabelRef = useRef(scaleLabel);

  const updateStatus = useCallback((next: string) => {
    if (statusRef.current === next) return;
    statusRef.current = next;
    setStatus(next);
  }, []);

  const updateGamepadName = useCallback((next: string) => {
    if (gamepadNameRef.current === next) return;
    gamepadNameRef.current = next;
    setGamepadName(next);
  }, []);

  const updateScaleLabel = useCallback((next: string) => {
    if (scaleLabelRef.current === next) return;
    scaleLabelRef.current = next;
    setScaleLabel(next);
  }, []);

  useEffect(() => {
    latestPoseRef.current = pose;
  }, [pose]);

  const syncServoLTarget = useCallback(() => {
    const current = poseToValues(latestPoseRef.current);
    targetPoseRef.current = current;
    updateStatus(current ? "ServoL target 已同步当前位姿" : "当前末端位姿不可用");
  }, [updateStatus]);

  const halt = useCallback(() => {
    onHalt();
    haltedRef.current = true;
  }, [onHalt]);

  useEffect(() => {
    if (!enabled) {
      halt();
      updateStatus("未启用");
      return;
    }

    let frame = 0;
    const intervalMs = 1000 / SEND_HZ;

    const tick = (time: number) => {
      const gamepads = navigator.getGamepads?.() ?? [];
      const gamepad = Array.from(gamepads).find(Boolean) ?? null;
      updateGamepadName(gamepad?.id ?? "");

      if (!gamepad) {
        updateStatus("未检测到手柄");
        halt();
        frame = window.requestAnimationFrame(tick);
        return;
      }

      if (lastTimeRef.current === 0) lastTimeRef.current = time;
      const dt = Math.min(0.1, Math.max(0, (time - lastTimeRef.current) / 1000));
      const elapsed = time - lastTimeRef.current;
      const buttons = gamepad.buttons.map((button) => button.pressed);
      const wasPressed = (index: number) => previousButtonsRef.current[index] === true;
      const rising = (index: number) => buttons[index] && !wasPressed(index);

      if (rising(BUTTON_START)) {
        setEnabled(false);
        halt();
        return;
      }

      if (rising(BUTTON_Y)) {
        setEnabled(false);
        halt();
        updateStatus("Y Halt");
        return;
      }

      if (rising(BUTTON_BACK)) syncServoLTarget();

      if (elapsed >= intervalMs) {
        const scale = pressed(gamepad, BUTTON_RB) ? FAST_SCALE : pressed(gamepad, BUTTON_LB) ? SLOW_SCALE : 1.0;
        updateScaleLabel(`${scale.toFixed(2)}x${pressed(gamepad, BUTTON_RB) ? " RB" : pressed(gamepad, BUTTON_LB) ? " LB" : ""}`);
        const twist = buildTwist(gamepad, scale);

        if (mode === "speedl") {
          onSend({ type: "speedl", twist, frame_id: "base_link" });
          updateStatus("SpeedL sending");
        } else {
          if (!targetPoseRef.current) {
            const current = poseToValues(latestPoseRef.current);
            targetPoseRef.current = current;
          }
          if (!targetPoseRef.current) {
            updateStatus("ServoL 需要当前末端位姿");
          } else {
            const target = targetPoseRef.current;
            target.x += twist[0] * dt;
            target.y += twist[1] * dt;
            target.z += twist[2] * dt;
            const delta = deltaQuaternion(twist[3], twist[4], twist[5], dt);
            const rotated = multiplyQuaternion(target, delta);
            targetPoseRef.current = normalizeQuaternion({ ...target, ...rotated });
            onSend({ type: "servol", ...targetPoseRef.current, frame_id: "base_link" });
            updateStatus("ServoL sending");
          }
        }

        haltedRef.current = false;
        lastTimeRef.current = time;
      }

      previousButtonsRef.current = buttons;
      frame = window.requestAnimationFrame(tick);
    };

    frame = window.requestAnimationFrame(tick);
    return () => {
      window.cancelAnimationFrame(frame);
      halt();
      lastTimeRef.current = 0;
    };
  }, [enabled, halt, mode, onSend, syncServoLTarget, updateGamepadName, updateScaleLabel, updateStatus]);

  useEffect(() => {
    const handleBlur = () => halt();
    const handleVisibility = () => {
      if (document.hidden) halt();
    };
    window.addEventListener("blur", handleBlur);
    document.addEventListener("visibilitychange", handleVisibility);
    return () => {
      window.removeEventListener("blur", handleBlur);
      document.removeEventListener("visibilitychange", handleVisibility);
    };
  }, [halt]);

  return (
    <div className="stream-card gamepad-card">
      <div className="subpanel-title">Xbox Gamepad</div>
      <div className="gamepad-status-grid">
        <span>device</span>
        <strong>{gamepadName || "未连接"}</strong>
        <span>mode</span>
        <strong>{mode === "speedl" ? "SpeedL" : "ServoL"}</strong>
        <span>scale</span>
        <strong>{scaleLabel}</strong>
        <span>status</span>
        <strong>{status}</strong>
      </div>
      <div className="gamepad-actions">
        <button className={enabled ? "danger-button" : ""} onClick={() => setEnabled((value) => !value)}>
          <Gamepad2 /> {enabled ? "Disable" : "Enable"}
        </button>
        <button className={mode === "speedl" ? "soft-active-button" : "ghost-button"} onClick={() => setMode("speedl")}>
          SpeedL
        </button>
        <button className={mode === "servol" ? "soft-active-button" : "ghost-button"} onClick={() => setMode("servol")}>
          ServoL
        </button>
        <button className="ghost-button" onClick={syncServoLTarget}>
          <RefreshCw /> Sync
        </button>
        <button className="ghost-button" onClick={halt}>
          <Pause /> Halt
        </button>
      </div>
      <div className="gamepad-help">
        Enable 后直接发送 · 左摇杆左右 X-/X+ · 左摇杆上下 Y+/Y- · 方向键上下 Z · 右摇杆左右绕 X · 右摇杆上下绕 Y · X/B yaw · Y halt
      </div>
    </div>
  );
}
