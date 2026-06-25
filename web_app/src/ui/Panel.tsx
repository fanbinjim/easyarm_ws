import type { ReactNode } from "react";

export function Panel({
  title,
  icon,
  children,
  className = "",
}: {
  title: string;
  icon: ReactNode;
  children: ReactNode;
  className?: string;
}) {
  return (
    <section className={`panel ${className}`}>
      <div className="panel-title">{icon}{title}</div>
      {children}
    </section>
  );
}
