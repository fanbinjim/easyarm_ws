# EasyArm Web App

React/Vite control panel for EasyArm motion debugging. This project is kept
outside ROS packages so it can later be packaged as a standalone desktop app or
served independently.

## Development

Install dependencies:

```bash
cd web_app
npm install
```

Start the ROS backend in another terminal:

```bash
cd ~/easyarm_ws
source install/setup.bash
export EASYARM_WEB_TOKEN=easyarm
ros2 launch easyarm_web_bridge web_bridge.launch.py
```

Start the web app:

```bash
cd web_app
npm run dev -- --host 127.0.0.1 --port 5173
```

Open `http://127.0.0.1:5173` and enter the token.

By default Vite proxies `/api` and `/ws` to `http://127.0.0.1:8000`. Override
the backend URL when needed:

```bash
EASYARM_WEB_BACKEND_URL=http://192.168.1.20:8000 npm run dev -- --host 0.0.0.0
```

For standalone builds that should call the backend directly:

```bash
VITE_EASYARM_API_BASE_URL=http://192.168.1.20:8000 npm run build
```
