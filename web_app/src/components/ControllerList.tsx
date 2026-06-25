import { ListChecks } from "lucide-react";
import { Panel } from "../ui/Panel";
import type { ControllerResponse } from "../api/types";

type Props = {
  controllers: ControllerResponse | null;
};

export function ControllerList({ controllers }: Props) {
  const items = controllers?.controllers ?? [];

  return (
    <Panel title="Controllers" icon={<ListChecks />}>
      {items.length === 0 ? (
        <div className="empty-hint">控制器数据未加载</div>
      ) : (
        <div className="controller-list">
          {items.map((c) => (
            <div className="controller-row" key={c.name}>
              <span>{c.name}</span>
              <strong>{c.state}</strong>
            </div>
          ))}
        </div>
      )}
    </Panel>
  );
}
