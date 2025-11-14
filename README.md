# L.Y.N.X.

## ROS 2 Autonomous Navigation Robot

A complete ROS 2 robot system for Raspberry Pi with autonomous navigation, camera-based object detection, and landmark recognition.

### Features

- 🤖 Autonomous navigation with A* path planning
- 📷 USB camera integration for real-time image processing
- 🎯 Object detection using OpenCV
- 🗺️ Map parsing and occupancy grid support
- 🚗 2-motor front-wheel steering control
- 🎮 Interactive waypoint selection interface

### Quick Start

See the [lynx_robot README](lynx_robot/README.md) for detailed installation and usage instructions.

### System Requirements

- Raspberry Pi 4 (or 3B+) with Raspberry Pi OS
- ROS 2 Humble or later
- USB Camera
- 2x DC Motors + Servo for steering
- Motor driver board

### Package Structure

```
lynx_robot/
├── lynx_robot/          # Python nodes
│   ├── motor_controller.py
│   ├── camera_node.py
│   ├── object_detector.py
│   ├── landmark_detector.py
│   ├── map_parser.py
│   ├── path_planner.py
│   ├── navigation_controller.py
│   └── waypoint_selector.py
├── launch/              # Launch files
├── config/              # Configuration files
├── maps/                # Map files
└── README.md           # Detailed documentation
```

### Quick Launch

```bash
# Install dependencies
cd lynx_robot
pip3 install -r requirements.txt

# Build the package
cd ~/ros2_ws
colcon build --packages-select lynx_robot
source install/setup.bash

# Launch the robot system
ros2 launch lynx_robot lynx_robot.launch.py
```

### License

MIT
