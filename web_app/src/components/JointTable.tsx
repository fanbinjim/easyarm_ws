import { useState } from "react";
import { SlidersHorizontal } from "lucide-react";
import { Panel } from "../ui/Panel";
import { numberText } from "../api/client";
import type { JointResponse } from "../api/types";

type Props = {
  joints: JointResponse | null;
  jointNames: string[];
};

export function JointTable({ joints, jointNames }: Props) {
  const [showDeg, setShowDeg] = useState(false);

  const convert = (rad: number) => showDeg ? rad * (180 / Math.PI) : rad;
  const unit = showDeg ? "°" : "rad";

  return (
    <Panel title="关节状态" icon={<SlidersHorizontal />}>
      <div className="joint-table-header">
        <button className="ghost-button unit-toggle" onClick={() => setShowDeg(!showDeg)}>
          {showDeg ? "deg → rad" : "rad → deg"}
        </button>
      </div>
      <div className="joint-table">
        <div className="joint-row joint-header">
          <span>关节</span>
          <strong>位置</strong>
          <small>速度</small>
        </div>
        {jointNames.map((name) => {
          const idx = joints?.names.indexOf(name) ?? -1;
          return (
            <div className="joint-row" key={name}>
              <span>{name}</span>
              <strong>{idx >= 0 ? convert(joints!.positions[idx]).toFixed(4) : "n/a"} {unit}</strong>
              <small>{idx >= 0 ? numberText(joints!.velocities[idx], 4) : "n/a"}</small>
            </div>
          );
        })}
        {jointNames.length === 0 && (
          <div className="empty-hint">关节数据未加载</div>
        )}
      </div>
    </Panel>
  );
}
