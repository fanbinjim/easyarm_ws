import { useRef, useState, useCallback } from "react";
import { NumberGrid } from "../ui/NumberGrid";
import { PoseEditor, DEFAULT_MOVEL } from "../ui/PoseEditor";
import { StreamButtons } from "../ui/StreamButtons";
import { api } from "../api/client";
import { Gauge } from "lucide-react";
import { GamepadControlPanel } from "./GamepadControlPanel";
import type { PoseResponse, StreamCommand } from "../api/types";

const JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"];
const DEFAULTS = { joints: [0, 1.85005, 2.68781, 0.9599, 1.57, 0] };

export function StreamPanel({ open: defaultOpen = false, pose }: { open?: boolean; pose: PoseResponse | null }) {
  const [open, setOpen] = useState(defaultOpen);
  const [speedJ, setSpeedJ] = useState([0, 0, 0, 0, 0, 0]);
  const [speedL, setSpeedL] = useState([0, 0, 0, 0, 0, 0]);
  const [servoJ, setServoJ] = useState(DEFAULTS.joints);
  const [servoL, setServoL] = useState(DEFAULT_MOVEL);
  const cmdRef = useRef(api.createCommandSocket());

  const halt = useCallback(() => {
    cmdRef.current.send({ type: "halt" });
  }, []);

  const sendStreamCommand = useCallback((command: StreamCommand) => {
    cmdRef.current.send(command);
  }, []);

  return (
    <details className="panel stream-section" open={open}>
      <summary
        onClick={(event) => {
          event.preventDefault();
          setOpen((value) => !value);
        }}
      >
        <span><Gauge /> 流式控制</span>
        <small>Speed / Servo 按住发送，松开 Halt</small>
      </summary>
      <div className="stream-grid">
        <div className="stream-card">
          <div className="subpanel-title">SpeedJ</div>
          <NumberGrid values={speedJ} onChange={setSpeedJ} labels={JOINT_NAMES} step={0.005} />
          <StreamButtons
            onStart={() => cmdRef.current.send({ type: "speedj", velocities: speedJ })}
            onStop={halt}
          />
        </div>

        <div className="stream-card">
          <div className="subpanel-title">SpeedL</div>
          <NumberGrid values={speedL} onChange={setSpeedL} labels={["vx", "vy", "vz", "wx", "wy", "wz"]} step={0.005} />
          <StreamButtons
            onStart={() => cmdRef.current.send({ type: "speedl", twist: speedL, frame_id: "base_link" })}
            onStop={halt}
          />
        </div>

        <div className="stream-card">
          <div className="subpanel-title">ServoJ</div>
          <NumberGrid values={servoJ} onChange={setServoJ} labels={JOINT_NAMES} step={0.01} />
          <StreamButtons
            onStart={() => cmdRef.current.send({ type: "servoj", joints: servoJ })}
            onStop={halt}
          />
        </div>

        <div className="stream-card">
          <div className="subpanel-title">ServoL</div>
          <PoseEditor value={servoL} onChange={setServoL} />
          <StreamButtons
            onStart={() => cmdRef.current.send({ type: "servol", ...servoL, frame_id: "base_link" })}
            onStop={halt}
          />
        </div>

        <GamepadControlPanel pose={pose} onSend={sendStreamCommand} onHalt={halt} />
      </div>
    </details>
  );
}
