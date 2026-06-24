import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

const backendUrl = process.env.EASYARM_WEB_BACKEND_URL ?? "http://127.0.0.1:8000";

export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      "/api": backendUrl,
      "/ws": {
        target: backendUrl.replace(/^http/, "ws"),
        ws: true
      }
    }
  }
});
