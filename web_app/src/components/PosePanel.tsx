import { Radio } from "lucide-react";
import { Panel } from "../ui/Panel";
import { Metric } from "../ui/Metric";
import { numberText } from "../api/client";
import type { PoseResponse } from "../api/types";

export function PosePanel({
  pose,
  onCopyToServoL,
}: {
  pose: PoseResponse | null;
  onCopyToServoL: () => void;
}) {
  return (
    <Panel title="末端位姿" icon={<Radio />}>
      <Metric label="frame" value={pose?.frame_id ?? "base_link"} />
      <Metric
        label="x y z (m)"
        value={`${numberText(pose?.position.x)} ${numberText(pose?.position.y)} ${numberText(pose?.position.z)}`}
      />
      <Metric
        label="qx qy qz qw"
        value={`${numberText(pose?.orientation.x)} ${numberText(pose?.orientation.y)} ${numberText(pose?.orientation.z)} ${numberText(pose?.orientation.w)}`}
      />
      <button className="ghost-button panel-action" onClick={onCopyToServoL}>
        复制到 ServoL
      </button>
    </Panel>
  );
}
