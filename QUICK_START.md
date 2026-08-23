# Kilin Hip Controller - Quick Start Guide

## TL;DR - Get Started in 5 Minutes

### 1. Build the packages
```bash
cd ~/kilin_ws
colcon build --packages-select kilin_msgs kilin_hip_controller kilin_cmd_converter --symlink-install
source install/setup.bash
```

### 2. Start the hip controller
```bash
ros2 launch kilin_hip_controller hip_controller.launch.py
```

### 3. Open web browser
Navigate to: `http://<robot-ip>:8080`

### 4. Control hips in real time
- Drag the 4 knobs (one for each hip)
- Or type exact values in radians
- Watch the robot respond!

## What Got Created?

### New ROS Package: `kilin_hip_controller`
- **Purpose**: Web UI for hip control
- **Location**: `~/kilin_ws/kilin_ros_ws/src/kilin_hip_controller/`
- **Publishes**: `/kilin/hip_cmd_position` with [FL, FR, RL, RR] hip angles

### Modified: `kilin_cmd_converter`
- **Change**: Now subscribes to hip commands
- **Behavior**: Merges web UI hip commands with regular wheel/steering commands
- **Result**: Full motor command to FPGA includes hip angles

## System Flow

```
Web Knob → HTTP → ROS Topic → kilin_cmd_converter → Motor Driver → Robot Hip
```

## Web UI Features

- 🎛️ **4 Interactive Knobs** - One for each hip module
- 📊 **Real-time Feedback** - See current position value
- ⌨️ **Direct Input** - Type exact radian values
- 🔄 **Reset Button** - Quick return to zero
- 🏠 **Home Button** - Set all to home position
- 📱 **Touch-friendly** - Works on tablets/phones
- 🌐 **Web-based** - Access from any device on network

## Common Commands

```bash
# Start with default port 8080
ros2 launch kilin_hip_controller hip_controller.launch.py

# Start with custom port
ros2 launch kilin_hip_controller hip_controller.launch.py web_port:=9000

# Direct node execution (for debugging)
ros2 run kilin_hip_controller kilin_hip_controller

# Monitor hip commands being published
ros2 topic echo /kilin/hip_cmd_position

# Monitor motor commands to FPGA
ros2 topic echo /kilin/motor_cmd_raw
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Web UI not loading | Check `http://<ip>:8080` - Make sure robot IP is correct |
| Hips not moving | Check if cmd_converter is also running: `ros2 node list` |
| Port already in use | Use different port: `-p web_port:=8081` |
| Build fails | Make sure you're in the kilin_ws directory and run `colcon build` |

## File Structure

```
~/kilin_ws/
└── kilin_ros_ws/src/
    ├── kilin_hip_controller/          [NEW]
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── README.md
    │   ├── src/
    │   │   └── kilin_hip_controller.cpp
    │   ├── resources/
    │   │   ├── index.html              [Web UI]
    │   │   ├── style.css               [Styling]
    │   │   └── script.js               [Interaction Logic]
    │   └── launch/
    │       └── hip_controller.launch.py
    │
    └── kilin_cmd_converter/            [MODIFIED]
        ├── src/
        │   └── kilin_cmd_converter.cpp [Added hip subscription]
        └── ...
```

## Control Range

Each hip can be set to any value in radians:
- **Minimum**: -2π radians (~-360°)
- **Maximum**: +2π radians (~+360°)
- **Recommended Range**: Usually -π to +π for normal operation

## Data Format

Hip positions are published as `Float64MultiArray`:
```cpp
data[0] = FL hip position (radians)
data[1] = FR hip position (radians)
data[2] = RL hip position (radians)
data[3] = RR hip position (radians)
```

## Next Steps

1. ✅ Build packages
2. ✅ Start hip controller  
3. ✅ Access web UI
4. 🔄 **Control hips in real time**
5. 📖 Read `INTEGRATION_GUIDE.md` for advanced features

## Support

For detailed information, see:
- `INTEGRATION_GUIDE.md` - Complete system documentation
- `README.md` (in kilin_hip_controller) - Package-specific docs
- `kilin_cmd_converter` source - How motor commands are merged

---

**Happy controlling! 🤖**
