import { Loader2 } from "lucide-react";
import { Panel } from "../ui/Panel";
import { Metric } from "../ui/Metric";
import type { Telemetry } from "../api/types";

type Props = {
  telemetry: Telemetry | null;
  onCancel: () => void;
};

export function ActionLog({ telemetry, onCancel }: Props) {
  const action = telemetry?.active_action;

  return (
    <Panel title="动作反馈" icon={<Loader2 />}>
      <Metric label="kind" value={action?.kind || "idle"} />
      <Metric label="state" value={action?.state ?? "idle"} />
      <Metric label="message" value={action?.message || "-"} />
      <div className="feedback-list">
        {(action?.feedback ?? []).slice(-6).map((item, i) => (
          <span key={`${i}-${item}`}>{item}</span>
        ))}
        {(!action?.feedback || action.feedback.length === 0) && (
          <span className="feedback-empty">暂无动作</span>
        )}
      </div>
      <button className="ghost-button panel-action" onClick={onCancel}>
        取消当前 action
      </button>
    </Panel>
  );
}
