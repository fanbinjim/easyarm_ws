import type { ReactNode } from "react";

export function StatusPill({
  label,
  tone,
  icon,
}: {
  label: string;
  tone: "good" | "warn" | "bad";
  icon: ReactNode;
}) {
  return (
    <span className={`status-pill ${tone}`}>
      {icon}
      {label}
    </span>
  );
}
