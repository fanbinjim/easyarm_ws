import { useCallback, useEffect, useRef, useState } from "react";
import * as THREE from "three";
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
const SEND_HZ = 30;
const SEND_INTERVAL_MS = 1000 / SEND_HZ;
const SEND_DT_SEC = 1 / SEND_HZ;
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

/**
 * 对摇杆输入做死区和平方曲线映射，降低零点漂移和低速抖动。
 */
function applyDeadzone(value: number): number {
  if (Math.abs(value) < DEADZONE) return 0;
  const normalized = (Math.abs(value) - DEADZONE) / (1 - DEADZONE);
  return Math.sign(value) * normalized * normalized;
}

/**
 * 安全读取指定手柄按钮状态，手柄未连接或按钮不存在时返回 false。
 */
function pressed(gamepad: Gamepad | null, index: number): boolean {
  return Boolean(gamepad?.buttons[index]?.pressed);
}

/**
 * 将后端末端位姿响应转换为前端 MoveL/ServoL 使用的扁平结构。
 */
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

/**
 * 将手柄角速度积分到 ServoL 目标姿态，返回新的目标位姿。
 *
 * `orientation.multiply(delta)` 保持当前语义为 target * delta，即在当前目标姿态上继续叠加增量旋转。
 */
function integratePoseOrientation(pose: PoseValues, wx: number, wy: number, wz: number, dt: number): PoseValues {
  const orientation = new THREE.Quaternion(pose.qx, pose.qy, pose.qz, pose.qw);
  const angle = Math.hypot(wx, wy, wz) * dt;

  if (angle >= 1e-9) {
    const axis = new THREE.Vector3(wx, wy, wz).normalize();
    const delta = new THREE.Quaternion().setFromAxisAngle(axis, angle);
    orientation.multiply(delta);
  }

  orientation.normalize();
  return {
    ...pose,
    qx: orientation.x,
    qy: orientation.y,
    qz: orientation.z,
    qw: orientation.w,
  };
}

/**
 * 按当前 Xbox 手柄映射生成笛卡尔速度指令：[vx, vy, vz, wx, wy, wz]。
 */
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

/**
 * Xbox 手柄控制面板，支持 SpeedL 速度控制和 ServoL 速度积分到目标位姿。
 */
export function GamepadControlPanel({ pose, onSend, onHalt }: Props) {
  const [enabled, setEnabled] = useState(false);
  const [mode, setMode] = useState<GamepadMode>("speedl");
  const [status, setStatus] = useState("未启用");
  const [gamepadName, setGamepadName] = useState("");
  const [scaleLabel, setScaleLabel] = useState("1.00x");
  const targetPoseRef = useRef<PoseValues | null>(null);
  const latestPoseRef = useRef<PoseResponse | null>(pose);
  const previousButtonsRef = useRef<boolean[]>([]);
  const haltedRef = useRef(true);
  const statusRef = useRef(status);
  const gamepadNameRef = useRef(gamepadName);
  const scaleLabelRef = useRef(scaleLabel);

  // 避免 30Hz 控制循环反复写入相同状态导致 React 重渲染。
  const updateStatus = useCallback((next: string) => {
    if (statusRef.current === next) return;
    statusRef.current = next;
    setStatus(next);
  }, []);

  // 只在手柄设备名变化时更新 UI。
  const updateGamepadName = useCallback((next: string) => {
    if (gamepadNameRef.current === next) return;
    gamepadNameRef.current = next;
    setGamepadName(next);
  }, []);

  // 只在倍率标签变化时更新 UI，保持发送循环轻量。
  const updateScaleLabel = useCallback((next: string) => {
    if (scaleLabelRef.current === next) return;
    scaleLabelRef.current = next;
    setScaleLabel(next);
  }, []);

  useEffect(() => {
    latestPoseRef.current = pose;
  }, [pose]);

  // 将 ServoL 的积分目标重置为机器人当前末端位姿。
  const syncServoLTarget = useCallback(() => {
    const current = poseToValues(latestPoseRef.current);
    targetPoseRef.current = current;
    updateStatus(current ? "ServoL target 已同步当前位姿" : "当前末端位姿不可用");
  }, [updateStatus]);

  // 统一发送 halt，并用 ref 避免重复发送 halt。
  const halt = useCallback(() => {
    if (haltedRef.current) return;
    onHalt();
    haltedRef.current = true;
  }, [onHalt]);

  useEffect(() => {
    if (!enabled) {
      halt();
      updateStatus("未启用");
      return;
    }

    let timer: ReturnType<typeof window.setInterval> | null = null;

    // 固定 30Hz 读取手柄并发送控制指令，让前后端命令节奏稳定。
    const tick = () => {
      const gamepads = navigator.getGamepads?.() ?? [];
      const gamepad = Array.from(gamepads).find(Boolean) ?? null;
      updateGamepadName(gamepad?.id ?? "");

      if (!gamepad) {
        updateStatus("未检测到手柄");
        halt();
        return;
      }

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
          target.x += twist[0] * SEND_DT_SEC;
          target.y += twist[1] * SEND_DT_SEC;
          target.z += twist[2] * SEND_DT_SEC;
          targetPoseRef.current = integratePoseOrientation(target, twist[3], twist[4], twist[5], SEND_DT_SEC);
          onSend({ type: "servol", ...targetPoseRef.current, frame_id: "base_link" });
          updateStatus("ServoL sending");
        }
      }

      haltedRef.current = false;
      previousButtonsRef.current = buttons;
    };

    tick();
    timer = window.setInterval(tick, SEND_INTERVAL_MS);
    return () => {
      if (timer) window.clearInterval(timer);
      halt();
    };
  }, [enabled, halt, mode, onSend, syncServoLTarget, updateGamepadName, updateScaleLabel, updateStatus]);

  // 页面失焦或切到后台时停止流式控制，避免后台继续下发运动指令。
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
