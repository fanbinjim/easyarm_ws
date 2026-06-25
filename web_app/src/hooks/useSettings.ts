import { useCallback, useEffect, useRef, useState } from "react";
import { getToken, setToken, getBaseUrl, setBaseUrl } from "../api/client";

export function useSettings() {
  const [token, setTokenState] = useState(getToken);
  const [baseUrl, setBaseUrlState] = useState(getBaseUrl);
  const [open, setOpen] = useState(false);
  const [draftToken, setDraftToken] = useState(token);
  const [draftUrl, setDraftUrl] = useState(baseUrl);

  const save = useCallback(() => {
    const t = draftToken.trim();
    const u = draftUrl.trim().replace(/\/$/, "");
    setToken(t);
    setBaseUrl(u);
    setTokenState(t);
    setBaseUrlState(u);
    setOpen(false);
  }, [draftToken, draftUrl]);

  const cancel = useCallback(() => {
    setDraftToken(token);
    setDraftUrl(baseUrl);
    setOpen(false);
  }, [token, baseUrl]);

  const openDialog = useCallback(() => {
    setDraftToken(token);
    setDraftUrl(baseUrl);
    setOpen(true);
  }, [token, baseUrl]);

  return {
    token,
    baseUrl,
    open,
    draftToken,
    draftUrl,
    setDraftToken,
    setDraftUrl,
    save,
    cancel,
    openDialog,
  };
}
