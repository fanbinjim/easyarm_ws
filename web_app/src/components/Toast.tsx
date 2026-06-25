import { createContext, useCallback, useContext, useState, type ReactNode } from "react";
import { X, AlertTriangle, CheckCircle, Info } from "lucide-react";

type ToastType = "error" | "warning" | "success" | "info";

type ToastItem = {
  id: number;
  type: ToastType;
  message: string;
  detail?: string;
};

type ToastContextValue = {
  toast: (type: ToastType, message: string, detail?: string) => void;
};

const ToastContext = createContext<ToastContextValue>({ toast: () => undefined });

let _nextId = 0;

export function ToastProvider({ children }: { children: ReactNode }) {
  const [toasts, setToasts] = useState<ToastItem[]>([]);

  const addToast = useCallback((type: ToastType, message: string, detail?: string) => {
    const id = ++_nextId;
    setToasts((prev) => [...prev.slice(-4), { id, type, message, detail }]);
    setTimeout(() => setToasts((prev) => prev.filter((t) => t.id !== id)), 8000);
  }, []);

  const dismiss = useCallback((id: number) => {
    setToasts((prev) => prev.filter((t) => t.id !== id));
  }, []);

  const iconMap: Record<ToastType, ReactNode> = {
    error: <AlertTriangle />,
    warning: <AlertTriangle />,
    success: <CheckCircle />,
    info: <Info />,
  };

  return (
    <ToastContext.Provider value={{ toast: addToast }}>
      {children}
      <div className="toast-container">
        {toasts.map((t) => (
          <div key={t.id} className={`toast toast-${t.type}`}>
            <span className="toast-icon">{iconMap[t.type]}</span>
            <div className="toast-body">
              <strong>{t.message}</strong>
              {t.detail && <small>{t.detail}</small>}
            </div>
            <button className="toast-dismiss" onClick={() => dismiss(t.id)}><X /></button>
          </div>
        ))}
      </div>
    </ToastContext.Provider>
  );
}

export function useToast(): ToastContextValue {
  return useContext(ToastContext);
}
