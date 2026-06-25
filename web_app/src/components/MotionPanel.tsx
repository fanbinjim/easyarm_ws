import { useEffect, useMemo, useState, type ReactNode } from "react";
import { Play, Send, ListChecks, Gauge } from "lucide-react";
import { Panel } from "../ui/Panel";
import { PoseEditor, type PoseValues } from "../ui/PoseEditor";
import { Range } from "../ui/Range";
import type { JointTarget, NamedStateResponse } from "../api/types";

const JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"];
const DEFAULT_MOVEJ = [0, 1.85005, 2.68781, 0.9599, 1.57, 0];
const JOINT_SLIDER_MIN = -3.14;
const JOINT_SLIDER_MAX = 3.14;

type Props = {
  namedStates: NamedStateResponse | null;
  planOnly: boolean;
  velocityScale: number;
  accelScale: number;
  onVelocityChange: (v: number) => void;
  onAccelChange: (v: number) => void;
  onPlanOnlyChange: (v: boolean) => void;
  onMoveJ: (joints: number[]) => void;
  onJointTargetChange: (target: JointTarget | null) => void;
  onMoveL: (pose: PoseValues) => void;
  onMoveLTargetChange: (pose: PoseValues | null) => void;
  onMoveNamedState: (name: string) => void;
  busy: boolean;
};

const TABS = [
  { key: "movej", label: "MoveJ", icon: <Send /> },
  { key: "movel", label: "MoveL", icon: <Send /> },
  { key: "named", label: "预设位姿", icon: <ListChecks /> },
] as const;

type MotionTab = (typeof TABS)[number]["key"];

export function MotionPanel({
  namedStates,
  planOnly,
  velocityScale,
  accelScale,
  onVelocityChange,
  onAccelChange,
  onPlanOnlyChange,
  onMoveJ,
  onJointTargetChange,
  onMoveL,
  onMoveLTargetChange,
  onMoveNamedState,
  busy,
}: Props) {
  const [tab, setTab] = useState<MotionTab>("movej");
  const [moveJValues, setMoveJValues] = useState(DEFAULT_MOVEJ);
  const [moveLValues, setMoveLValues] = useState({ x: 0.25, y: 0, z: 0.25, qx: 0, qy: 0, qz: 0, qw: 1 });
  const [selectedNamed, setSelectedNamed] = useState("");

  const namedStatesList = namedStates?.states ?? [];
  const activeNamedValues = useMemo(
    () => namedStatesList.find((s) => s.name === selectedNamed)?.positions ?? [],
    [namedStatesList, selectedNamed],
  );

  const actionLabel = planOnly ? "规划" : "执行";

  useEffect(() => {
    onMoveLTargetChange(tab === "movel" ? moveLValues : null);
  }, [moveLValues, onMoveLTargetChange, tab]);

  useEffect(() => {
    if (tab === "movej") {
      onJointTargetChange({ names: JOINT_NAMES, positions: moveJValues });
      return;
    }

    if (tab === "named" && activeNamedValues.length > 0) {
      onJointTargetChange({ names: namedStates?.joint_names ?? JOINT_NAMES, positions: activeNamedValues });
      return;
    }

    onJointTargetChange(null);
  }, [activeNamedValues, moveJValues, namedStates?.joint_names, onJointTargetChange, tab]);

  useEffect(() => {
    if (!selectedNamed && namedStatesList.length > 0) {
      setSelectedNamed(namedStatesList[0].name);
    }
  }, [namedStatesList, selectedNamed]);

  const tabContent: Record<MotionTab, ReactNode> = {
    movej: (
      <div className="motion-tab-content">
        <JointSliderGrid values={moveJValues} onChange={setMoveJValues} />
        <button disabled={busy} onClick={() => onMoveJ(moveJValues)}>
          <Play /> {actionLabel} MoveJ
        </button>
      </div>
    ),
    movel: (
      <div className="motion-tab-content">
        <PoseEditor value={moveLValues} onChange={setMoveLValues} />
        <button disabled={busy} onClick={() => onMoveL(moveLValues)}>
          <Play /> {actionLabel} MoveL
        </button>
      </div>
    ),
    named: (
      <div className="motion-tab-content">
        <select value={selectedNamed} onChange={(e) => setSelectedNamed(e.target.value)}>
          <option value="" disabled>选择一个姿态</option>
          {namedStatesList.map((s) => (
            <option key={s.name} value={s.name}>{s.name}</option>
          ))}
        </select>
        <div className="named-values">
          {activeNamedValues.length > 0
            ? activeNamedValues.map((v) => v.toFixed(4)).join(" ")
            : "未选择"}
        </div>
        <button disabled={busy || !selectedNamed} onClick={() => onMoveNamedState(selectedNamed)}>
          <Play /> {actionLabel} 预设位姿
        </button>
      </div>
    ),
  };

  return (
    <Panel title="运动控制台" icon={<Gauge />} className="control-console">
      <div className="control-head">
        <div className="mode-switch" role="group" aria-label="运动命令模式">
          <button type="button" className={planOnly ? "active" : ""} onClick={() => onPlanOnlyChange(true)}>
            规划
          </button>
          <button type="button" className={!planOnly ? "active execute" : ""} onClick={() => onPlanOnlyChange(false)}>
            执行
          </button>
        </div>
        <div className={`mode-note ${planOnly ? "safe" : "danger"}`}>
          {planOnly ? "只做 MoveIt 规划，不执行真实运动" : "执行模式会下发真实运动，请确认环境安全"}
        </div>
      </div>

      <div className="parameter-grid">
        <Range label="velocity" value={velocityScale} setValue={onVelocityChange} />
        <Range label="acceleration" value={accelScale} setValue={onAccelChange} />
      </div>

      <div className="motion-tabs" role="tablist">
        {TABS.map((t) => (
          <button
            key={t.key}
            role="tab"
            className={`motion-tab ${tab === t.key ? "active" : ""}`}
            onClick={() => setTab(t.key)}
          >
            {t.icon}
            {t.label}
          </button>
        ))}
      </div>

      {tabContent[tab]}
    </Panel>
  );
}

function JointSliderGrid({
  values,
  onChange,
}: {
  values: number[];
  onChange: (value: number[]) => void;
}) {
  const setValue = (index: number, value: number) => {
    const next = [...values];
    next[index] = value;
    onChange(next);
  };

  return (
    <div className="joint-slider-grid">
      {values.map((value, index) => (
        <label className="joint-slider-row" key={JOINT_NAMES[index]}>
          <span>{JOINT_NAMES[index]}</span>
          <input
            type="range"
            min={JOINT_SLIDER_MIN}
            max={JOINT_SLIDER_MAX}
            step="0.001"
            value={value}
            onChange={(event) => setValue(index, Number(event.target.value))}
          />
          <input
            type="number"
            min={JOINT_SLIDER_MIN}
            max={JOINT_SLIDER_MAX}
            step="0.001"
            value={Number.isFinite(value) ? value : 0}
            onChange={(event) => setValue(index, Number(event.target.value))}
          />
        </label>
      ))}
    </div>
  );
}
