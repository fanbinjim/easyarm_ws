import { Lock, X } from "lucide-react";

export function SettingsDialog({
  draftToken,
  setDraftToken,
  draftUrl,
  setDraftUrl,
  onSave,
  onCancel,
}: {
  draftToken: string;
  setDraftToken: (v: string) => void;
  draftUrl: string;
  setDraftUrl: (v: string) => void;
  onSave: () => void;
  onCancel: () => void;
}) {
  return (
    <div className="confirm-backdrop" role="presentation" onMouseDown={onCancel}>
      <div
        className="confirm-dialog settings-dialog"
        role="dialog"
        aria-modal="true"
        onMouseDown={(event) => event.stopPropagation()}
      >
        <div className="confirm-icon warn">
          <Lock />
        </div>
        <div className="confirm-content">
          <h2>连接设置</h2>
          <div className="settings-fields">
            <label>
              <span>Token</span>
              <input
                type="password"
                value={draftToken}
                onChange={(e) => setDraftToken(e.target.value)}
                placeholder="easyarm"
              />
            </label>
            <label>
              <span>Backend URL</span>
              <input
                value={draftUrl}
                onChange={(e) => setDraftUrl(e.target.value)}
                placeholder="http://127.0.0.1:8000"
              />
            </label>
            <small>修改后保存将立即生效。留空后端 URL 则走 Vite 代理。</small>
          </div>
          <div className="confirm-actions">
            <button className="ghost-button" onClick={onCancel}>取消</button>
            <button className="soft-active-button" onClick={onSave}>保存</button>
          </div>
        </div>
      </div>
    </div>
  );
}
