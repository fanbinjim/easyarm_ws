import { useCallback, useEffect, useRef, useState } from "react";
import { api, ApiError, isRecoverableError, recoverableMessage, errorMessage } from "../api/client";
import type { HealthResponse, MotionServerStatus, BackendStatus } from "../api/types";

const HEALTH_INTERVAL_MS = 1500;

export type HealthSnapshot = {
  data: HealthResponse | null;
  backendStatus: BackendStatus;
  motionServerStatus: MotionServerStatus;
  isMockHardware: "true" | "false" | "unknown";
  fatalError: string;
  warning: string;
};

export function useHealth(token: string): HealthSnapshot {
  const [data, setData] = useState<HealthResponse | null>(null);
  const [backendStatus, setBackendStatus] = useState<BackendStatus>("disconnected");
  const [motionServerStatus, setMotionServerStatus] = useState<MotionServerStatus>("unavailable");
  const [isMockHardware, setIsMockHardware] = useState<"true" | "false" | "unknown">("unknown");
  const [fatalError, setFatalError] = useState("");
  const [warning, setWarning] = useState("");
  const authRef = useRef(false);

  const poll = useCallback(async () => {
    if (!token) {
      setBackendStatus("disconnected");
      setData(null);
      return;
    }

    try {
      const healthData = await api.health;
      setData(healthData);
      setBackendStatus("connected");

      const mockVal = healthData.is_mock_hardware === "true" ? "true" : healthData.is_mock_hardware === "false" ? "false" : "unknown";
      setIsMockHardware(mockVal);

      if (!healthData.motion_server?.get_state) {
        setMotionServerStatus("unavailable");
      } else if (!healthData.motion_server?.movej || !healthData.motion_server?.movel || !healthData.motion_server?.move_named_state) {
        setMotionServerStatus("degraded");
      } else {
        setMotionServerStatus("ready");
      }

      setFatalError("");
      setWarning("");
      authRef.current = false;
    } catch (err) {
      if (err instanceof ApiError && err.isUnauthorized) {
        setBackendStatus("unauthorized");
        setFatalError(errorMessage(err));
        authRef.current = true;
      } else {
        setBackendStatus("error");
        if (isRecoverableError(err)) {
          const msg = recoverableMessage(err);
          if (msg) setWarning(msg);
        } else {
          setFatalError(errorMessage(err));
        }
      }
    }
  }, [token]);

  useEffect(() => {
    poll();
    if (authRef.current) return;
    const interval = window.setInterval(poll, HEALTH_INTERVAL_MS);
    return () => window.clearInterval(interval);
  }, [poll]);

  return { data, backendStatus, motionServerStatus, isMockHardware, fatalError, warning };
}
