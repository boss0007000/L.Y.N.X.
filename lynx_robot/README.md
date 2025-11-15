# L.Y.N.X. Robot - ROS 2 Autonomous Navigation System

A complete ROS 2 robot system designed for Raspberry Pi running Raspberry Pi OS. The system provides autonomous navigation with camera-based object and landmark detection using a 2-motor front-wheel steering robot.

## Features

- **Autonomous Navigation**: A* path planning algorithm for efficient route finding
- **Map Support**: Load and parse occupancy grid maps
- **Interactive Waypoint Selection**: Set start and goal positions for navigation
- **USB Camera Integration**: Real-time image capture and processing
- **Object Detection**: Color-based and contour-based object detection using OpenCV
- **Landmark Detection**: ArUco markers, QR codes, and corner detection
- **Motor Control**: Front-wheel steering with 2-motor drive system
- **Pure Pursuit Controller**: Smooth trajectory following

## System Architecture

The system consists of the following ROS 2 nodes:

1. **motor_controller**: Controls the robot's motors and steering
2. **camera_node**: Captures images from USB camera
3. **object_detector**: Detects objects in camera images
4. **landmark_detector**: Detects landmarks (ArUco, QR codes, etc.)
5. **map_parser**: Loads and publishes map data
6. **path_planner**: Plans paths using A* algorithm
7. **navigation_controller**: Executes planned paths
8. **waypoint_selector**: Interactive interface for selecting waypoints

## Hardware Requirements

- Raspberry Pi 4 (recommended) or Raspberry Pi 3B+
- Raspberry Pi OS (Bullseye or later)
- USB Camera
- 2x DC Motors with motor driver
- Servo motor for steering
- Motor driver board (e.g., L298N)
- 4x HC-SR04 Ultrasonic sensors
- Power supply (7.4V LiPo recommended)

### GPIO Pin Configuration (Default)

**Complete GPIO Pin Assignment (BCM Numbering):**

| GPIO Pin | Function | Hardware Component | Notes |
|----------|----------|-------------------|-------|
| GPIO 1   | Echo     | Ultrasonic Sensor (Left) | Use voltage divider (5V→3.3V) |
| GPIO 7   | Trigger  | Ultrasonic Sensor (Left) | |
| GPIO 8   | Echo     | Ultrasonic Sensor (Back) | Use voltage divider (5V→3.3V) |
| GPIO 12  | Trigger  | Ultrasonic Sensor (Right) | |
| GPIO 16  | Echo     | Ultrasonic Sensor (Right) | Use voltage divider (5V→3.3V) |
| GPIO 17  | PWM      | Left Motor (via motor driver) | Hardware PWM capable |
| GPIO 18  | PWM      | Right Motor (via motor driver) | Hardware PWM capable |
| GPIO 23  | Trigger  | Ultrasonic Sensor (Front) | |
| GPIO 24  | Echo     | Ultrasonic Sensor (Front) | Use voltage divider (5V→3.3V) |
| GPIO 25  | Trigger  | Ultrasonic Sensor (Back) | |
| GPIO 27  | PWM      | Steering Servo Motor | Hardware PWM for servo |

**Configuration Files:**
- Motors: `config/motor_config.yaml`
- Ultrasonic Sensors: `config/ultrasonic_config.yaml`

**⚠️ Important:** HC-SR04 ECHO pins output 5V. Use voltage dividers (1kΩ and 2kΩ resistors) to reduce to 3.3V before connecting to Raspberry Pi GPIO pins.

For detailed wiring diagrams, see [HARDWARE.md](../HARDWARE.md) in the repository root.

## Software Requirements

- ROS 2 Humble (or later)
- Python 3.8+
- OpenCV
- NumPy

## Installation

### 1. Install ROS 2

Follow the official ROS 2 installation guide for Ubuntu/Debian:
https://docs.ros.org/en/humble/Installation.html

For Raspberry Pi OS, you may need to build from source or use Docker.

### 2. Install Dependencies

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-yaml python3-pil
sudo apt install -y ros-humble-cv-bridge ros-humble-image-transport
```

### 3. Install GPIO Library (for Raspberry Pi)

```bash
sudo apt install -y python3-rpi.gpio
```

### 4. Create ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 5. Clone or Copy the Package

```bash
# If cloning from repository
git clone https://github.com/boss0007000/L.Y.N.X.git
cd L.Y.N.X.

# Or copy the lynx_robot package to your workspace
cp -r /path/to/lynx_robot ~/ros2_ws/src/
```

### 6. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select lynx_robot
source install/setup.bash
```

## Usage

### Launch the Complete System

```bash
ros2 launch lynx_robot lynx_robot.launch.py
```

This will start all nodes including motor control, camera, detection, navigation, and the interactive waypoint selector.

### Launch Without Camera (for testing)

```bash
ros2 launch lynx_robot lynx_robot.launch.py use_camera:=false
```

### Using the Waypoint Selector

Once the system is running, you'll see an interactive prompt:

```
Commands:
  s <x> <y>  - Set start position
  g <x> <y>  - Set goal position
  p          - Plan path with current start and goal
  r          - Reset start and goal
  q          - Quit

Example:
s 0 0        # Set start at origin
g 5 5        # Set goal at (5, 5) meters
p            # Plan and execute path
```

### Running Individual Nodes

You can also run nodes individually for testing:

```bash
# Camera node
ros2 run lynx_robot camera_node

# Object detector
ros2 run lynx_robot object_detector

# Motor controller
ros2 run lynx_robot motor_controller

# Map parser
ros2 run lynx_robot map_parser --ros-args -p map_file:=/path/to/map.yaml

# Path planner
ros2 run lynx_robot path_planner

# Navigation controller
ros2 run lynx_robot navigation_controller

# Waypoint selector
ros2 run lynx_robot waypoint_selector
```

## Configuration

Configuration files are located in the `config/` directory:

- **motor_config.yaml**: Motor and steering parameters
- **camera_config.yaml**: Camera settings
- **detection_config.yaml**: Object detection parameters
- **landmark_config.yaml**: Landmark detection settings
- **map_config.yaml**: Map loading parameters
- **planner_config.yaml**: Path planning settings
- **navigation_config.yaml**: Navigation controller parameters

Edit these files to customize the robot's behavior.

## Creating Maps

### Option 1: Use Existing Maps

Place your map files (YAML + PGM/PNG) in the `maps/` directory and reference them in the launch file.

### Option 2: Create Custom Maps

Maps should be in ROS occupancy grid format:
- **YAML file**: Contains metadata (resolution, origin, thresholds)
- **Image file** (PGM/PNG): Grayscale image where:
  - White (255) = free space
  - Black (0) = occupied
  - Gray = unknown

Example map.yaml:
```yaml
image: my_map.pgm
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### Option 3: Use SLAM

Use ROS 2 SLAM packages like `slam_toolbox` to create maps:

```bash
sudo apt install ros-humble-slam-toolbox
ros2 launch slam_toolbox online_async_launch.py
```

## ROS 2 Topics

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist): Velocity commands to motors
- `/camera/image_raw` (sensor_msgs/Image): Raw camera images
- `/camera/camera_info` (sensor_msgs/CameraInfo): Camera calibration
- `/detected_objects` (std_msgs/String): Detected objects (JSON)
- `/detected_landmarks` (std_msgs/String): Detected landmarks (JSON)
- `/map` (nav_msgs/OccupancyGrid): Map data
- `/planned_path` (nav_msgs/Path): Planned path
- `/motor_commands` (std_msgs/Float32MultiArray): Motor control signals

### Subscribed Topics

- `/planning_request` (std_msgs/String): Path planning requests (JSON)
- `/odom` (nav_msgs/Odometry): Robot odometry
- `/current_pose` (geometry_msgs/PoseStamped): Current robot pose

## Debugging

### View Camera Feed

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/camera/image_raw` to view the camera stream.

### View Detection Results

```bash
ros2 topic echo /detected_objects
ros2 topic echo /detected_landmarks
```

### Visualize Navigation

```bash
# Install RViz2
sudo apt install ros-humble-rviz2

# Run RViz2
ros2 run rviz2 rviz2
```

Add displays for:
- Map (`/map`)
- Path (`/planned_path`)
- Camera (`/camera/image_raw`)

### Monitor System Status

```bash
# List all running nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor topic data
ros2 topic echo /navigation_status
ros2 topic echo /planning_status
```

## Troubleshooting

### Camera Not Found

- Check USB connection: `ls /dev/video*`
- Test with: `v4l2-ctl --list-devices`
- Adjust `camera_index` parameter in `camera_config.yaml`

### GPIO Permissions

If you get GPIO permission errors:
```bash
sudo usermod -a -G gpio $USER
# Log out and log back in
```

### Motors Not Responding

- Check GPIO pin connections
- Verify motor driver power supply
- Test GPIO: `gpio readall` (install with `sudo apt install wiringpi`)
- Check motor configuration in `motor_config.yaml`

### No Path Found

- Verify map is loaded: `ros2 topic echo /map_info`
- Check start/goal positions are in free space
- Increase `goal_tolerance` in `planner_config.yaml`

## Development

### Adding Custom Detection Algorithms

Modify `object_detector.py` or `landmark_detector.py` to add custom detection methods:

```python
def detect_custom(self, image):
    # Your detection code here
    detections = []
    # ...
    return detections
```

### Modifying Path Planning

Edit `path_planner.py` to implement different algorithms (Dijkstra, RRT, etc.):

```python
def plan_custom(self, start, goal):
    # Your planning algorithm
    path = []
    # ...
    return path
```

## Performance Optimization

### Raspberry Pi 4

- Default settings work well
- Consider overclocking for better performance

### Raspberry Pi 3

- Reduce camera resolution: `image_width: 320, image_height: 240`
- Lower frame rate: `frame_rate: 15`
- Disable debug images: `publish_debug_image: false`
- Reduce control frequency: `control_frequency: 5.0`

## Safety Features

- Maximum speed limits configurable
- Obstacle avoidance through map-based planning
- Emergency stop: Press Ctrl+C to stop all nodes
- Manual override: Publish to `/cmd_vel` directly

## License

MIT License

## Contributing

Contributions are welcome! Please open issues or pull requests on GitHub.

## Support

For questions and support:
- Open an issue on GitHub
- Check ROS 2 documentation: https://docs.ros.org

## Acknowledgments

- ROS 2 Community
- OpenCV Project
- Raspberry Pi Foundation
