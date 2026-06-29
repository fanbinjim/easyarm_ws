import { useState } from "react";
import { AlertTriangle } from "lucide-react";
import type { Telemetry } from "../api/types";

type Props = {
  telemetry: Telemetry | null;
};

export function RosLog({ telemetry }: Props) {
  const [open, setOpen] = useState(false);
  const logs = telemetry?.rosout ?? [];

  return (
    <details className="panel stream-section log-panel" open={open}>
      <summary
        onClick={(event) => {
          event.preventDefault();
          setOpen((value) => !value);
        }}
      >
        <span><AlertTriangle /> ROS 日志</span>
        <small>{logs.length > 0 ? `${logs.length} 条` : "暂无日志"}</small>
      </summary>
      <div className="log-panel-body">
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
      </div>
    </details>
  );
}
