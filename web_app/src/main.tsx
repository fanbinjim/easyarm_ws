import React from "react";
import { createRoot } from "react-dom/client";
import App from "./App";
import "./styles.css";

const rootElement = document.getElementById("root");
if (!rootElement) {
  throw new Error("root element not found");
}

const app = <App />;

createRoot(rootElement).render(
  import.meta.env.DEV ? app : <React.StrictMode>{app}</React.StrictMode>,
);
