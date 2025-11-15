# L.Y.N.X. - Learning Your Navigation eXpertise

## What This Project Does

**L.Y.N.X.** is a complete autonomous navigation robot system built with ROS 2 (Robot Operating System) that runs on a Raspberry Pi. This project enables a physical robot to navigate autonomously from a start position to a goal position while avoiding obstacles and detecting objects in its environment.

### Core Capabilities

**The robot can:**
- **Navigate autonomously** using A* path planning algorithm on pre-loaded maps
- **Detect and avoid obstacles** using ultrasonic sensors (4 sensors: front, back, left, right)
- **Recognize objects** in real-time using a USB camera and OpenCV-based color/contour detection
- **Detect landmarks** including ArUco markers, QR codes, and corner features for localization
- **Control physical motors** with a 2-motor front-wheel steering system via Raspberry Pi GPIO
- **Plan optimal paths** on occupancy grid maps while avoiding obstacles
- **Provide interactive control** through both visual map-based and text-based interfaces
- **Monitor its environment** continuously through sensor fusion (camera + ultrasonics)

### How It Works

1. **You provide a map** (occupancy grid) showing free space and obstacles
2. **You select start and goal positions** using the visual or text interface
3. **The robot plans an optimal path** using A* algorithm
4. **The robot navigates autonomously** following the planned path with Pure Pursuit control
5. **Sensors monitor the environment** detecting objects and landmarks in real-time
6. **Motors respond to navigation commands** steering and driving the robot to its goal

### Key Features

- 🤖 **Autonomous navigation** with A* path planning algorithm
- 📷 **USB camera integration** for real-time image processing (640x480 @ 30fps)
- 🎯 **Object detection** using OpenCV color and contour-based detection
- 🏷️ **Landmark recognition** with ArUco markers, QR codes, and corner detection
- 📡 **Ultrasonic sensors** for proximity detection and obstacle avoidance
- 🗺️ **Map-based navigation** supporting occupancy grid maps (PNG/PGM format)
- 🚗 **2-motor front-wheel steering** with GPIO-controlled DC motors and servo
- 🎮 **Interactive waypoint selection** with visual map display or text interface
- 🔧 **Fully configurable** with YAML configuration files for all components
- 🛠️ **Modular ROS 2 architecture** with 9 independent nodes that work together

### Quick Start

See the [lynx_robot README](lynx_robot/README.md) for detailed installation and usage instructions.

### System Requirements

**Hardware:**
- Raspberry Pi 4 (recommended) or 3B+ with Raspberry Pi OS
- USB Camera (any standard webcam)
- 2x DC Motors for rear-wheel drive
- 1x Servo motor for front-wheel steering
- Motor driver board (e.g., L298N or similar)
- 4x HC-SR04 Ultrasonic sensors (front, back, left, right)
- Power supply (7.4V for motors, 5V for Raspberry Pi)

**Software:**
- ROS 2 Humble or later
- Python 3.8+
- OpenCV, NumPy, PyYAML, Pillow

### Package Structure

```
lynx_robot/
├── lynx_robot/          # Python ROS 2 nodes
│   ├── motor_controller.py         # GPIO motor control
│   ├── camera_node.py               # USB camera capture
│   ├── object_detector.py           # OpenCV-based object detection
│   ├── landmark_detector.py         # ArUco/QR code detection
│   ├── ultrasonic_sensor.py         # HC-SR04 sensor reading
│   ├── map_parser.py                # Occupancy grid map loader
│   ├── path_planner.py              # A* path planning
│   ├── navigation_controller.py     # Pure Pursuit path following
│   └── waypoint_selector.py         # Interactive UI for goal setting
├── launch/              # ROS 2 launch files
├── config/              # YAML configuration files
├── maps/                # Occupancy grid map files
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
