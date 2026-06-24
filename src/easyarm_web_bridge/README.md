# easyarm_web_bridge

ROS 2 backend bridge for the EasyArm web app. It exposes a small HTTP/WebSocket
API and keeps motion execution behind `easyarm_motion_server`.

The React/Vite frontend lives in the workspace root at `web_app/`.

## Development

Install runtime dependencies:

```bash
sudo apt-get install python3-fastapi python3-uvicorn
```

Build and source the workspace:

```bash
cd ~/easyarm_ws
colcon build --packages-select easyarm_web_bridge easyarm_a1_bringup
source install/setup.bash
```

Start only the web backend for frontend debugging:

```bash
export EASYARM_WEB_TOKEN=easyarm
ros2 launch easyarm_web_bridge web_bridge.launch.py
```

You can also pass parameters explicitly:

```bash
ros2 launch easyarm_web_bridge web_bridge.launch.py \
  host:=127.0.0.1 \
  port:=8000 \
  token:=easyarm
```

Later, start the backend with the robot bringup:

```bash
ros2 launch easyarm_a1_bringup bringup.launch.py \
  use_mock_hardware:=true \
  moveit_servo:=true \
  web:=true \
  web_token:=easyarm
```
