import { useEffect, useRef, useState } from "react";
import { api } from "../api/client";
import type { Telemetry, TelemetryFreshness } from "../api/types";

const STALE_THRESHOLD_MS = 3000;

export type TelemetrySnapshot = {
  data: Telemetry | null;
  freshness: TelemetryFreshness;
  connected: boolean;
};

export function useTelemetry(token: string): TelemetrySnapshot {
  const [data, setData] = useState<Telemetry | null>(null);
  const [freshness, setFreshness] = useState<TelemetryFreshness>("missing");
  const [connected, setConnected] = useState(false);
  const socketRef = useRef<ReturnType<typeof api.createTelemetrySocket> | null>(null);
  const lastMsgRef = useRef(0);
  const timerRef = useRef<ReturnType<typeof setInterval> | null>(null);

  useEffect(() => {
    if (!token) {
      setConnected(false);
      setFreshness("missing");
      return;
    }

    const socket = api.createTelemetrySocket((msg) => {
      setData(msg);
      setConnected(true);
      setFreshness("fresh");
      lastMsgRef.current = Date.now();
    });

    socketRef.current = socket;
    socket.connect();

    timerRef.current = setInterval(() => {
      const elapsed = Date.now() - lastMsgRef.current;
      if (elapsed > STALE_THRESHOLD_MS && lastMsgRef.current > 0) {
        setFreshness("stale");
        setConnected(false);
      }
    }, 1000);

    const handleVisibility = () => {
      if (document.hidden) return;
      if (socketRef.current?.readyState !== WebSocket.OPEN) {
        socket.connect();
      }
    };

    document.addEventListener("visibilitychange", handleVisibility);

    return () => {
      socket.disconnect();
      socketRef.current = null;
      if (timerRef.current) clearInterval(timerRef.current);
      document.removeEventListener("visibilitychange", handleVisibility);
    };
  }, [token]);

  return { data, freshness, connected };
}
