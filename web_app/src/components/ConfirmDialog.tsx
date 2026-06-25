import { useEffect } from "react";
import { AlertTriangle } from "lucide-react";

export type ConfirmDialogState = {
  title: string;
  message: string;
  confirmLabel: string;
  tone: "danger" | "warn";
  onConfirm: () => void;
};

export function ConfirmDialog({
  dialog,
  onClose,
}: {
  dialog: ConfirmDialogState;
  onClose: () => void;
}) {
  useEffect(() => {
    const handler = (event: KeyboardEvent) => {
      if (event.key === "Escape") onClose();
    };
    window.addEventListener("keydown", handler);
    return () => window.removeEventListener("keydown", handler);
  }, [onClose]);

  return (
    <div className="confirm-backdrop" role="presentation" onMouseDown={onClose}>
      <div
        className="confirm-dialog"
        role="dialog"
        aria-modal="true"
        aria-labelledby="confirm-title"
        onMouseDown={(event) => event.stopPropagation()}
      >
        <div className={`confirm-icon ${dialog.tone}`}>
          <AlertTriangle />
        </div>
        <div className="confirm-content">
          <h2 id="confirm-title">{dialog.title}</h2>
          <p>{dialog.message}</p>
          <div className="confirm-actions">
            <button className="ghost-button" onClick={onClose}>取消</button>
            <button
              className={dialog.tone === "danger" ? "danger-button" : "soft-active-button"}
              onClick={() => {
                dialog.onConfirm();
                onClose();
              }}
            >
              {dialog.confirmLabel}
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
