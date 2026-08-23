# Kilin Hip Controller

A web-based UI controller for real-time hip position control with 4 interactive knobs.

## Features

- **Web-Based Interface**: Access the UI from any device with a web browser (supports touch screens)
- **Real-Time Control**: 4 interactive knobs for controlling FL, FR, RL, RR hip positions
- **Range Control**: Each knob controls hip position in range of -2π to +2π radians
- **Integration**: Publishes hip position commands to `/kilin/hip_cmd_position` topic

## Architecture

### Communication Flow

```
Web UI (HTML/JS)
    ↓
HTTP Server (cpp-httplib)
    ↓
ROS 2 Node (kilin_hip_controller)
    ↓
/kilin/hip_cmd_position (Float64MultiArray)
    ↓
kilin_cmd_converter (subscribes & merges with wheel/steering)
    ↓
/kilin/motor_cmd_raw
```

## Building

```bash
cd ~/kilin_ws
colcon build --packages-select kilin_hip_controller
```

## Running

```bash
ros2 run kilin_hip_controller kilin_hip_controller --ros-args -p web_port:=8080
```

Then access the web UI at: `http://<robot-ip>:8080`

## Parameters

- `web_port` (int, default: 8080): Port for the web server
- `resources_dir` (string, default: auto-detect): Directory containing web resources (HTML/CSS/JS)

## API Endpoints

- `GET /api/health`: Health check
- `POST /api/hip/set`: Set a hip position
  - Body: `{"module": "A|B|C|D", "position": <value>}`
- `GET /api/hip/get`: Get current hip positions

## Topic

- **Published**: `/kilin/hip_cmd_position` (std_msgs/Float64MultiArray)
  - Data: [hip_A, hip_B, hip_C, hip_D] in radians
