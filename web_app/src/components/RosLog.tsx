import { AlertTriangle } from "lucide-react";
import type { Telemetry } from "../api/types";

type Props = {
  telemetry: Telemetry | null;
};

export function RosLog({ telemetry }: Props) {
  const logs = telemetry?.rosout ?? [];

  return (
    <section className="panel log-panel">
      <div className="panel-title"><AlertTriangle /> ROS 日志</div>
      {logs.length === 0 ? (
        <div className="empty-hint">暂无日志</div>
      ) : (
        <div className="log-list">
          {logs.slice(-12).map((item, i) => (
            <div key={`${item.name}-${i}-${item.stamp?.sec ?? 0}`}>
              <strong>{item.name}</strong>
              <span>{item.message}</span>
            </div>
          ))}
        </div>
      )}
    </section>
  );
}
