# Quick Start Guide for L.Y.N.X. Robot

This guide will help you get your L.Y.N.X. robot up and running quickly.

## Prerequisites

- Raspberry Pi 4 (or 3B+) with Raspberry Pi OS installed
- USB Camera plugged in
- Motors and motor driver connected
- Power supply connected

## Installation

### Option 1: Automated Installation (Recommended)

```bash
cd /path/to/L.Y.N.X.
bash install.sh
```

The script will:
- Install all required dependencies
- Set up ROS 2 packages
- Configure GPIO permissions
- Build the package

### Option 2: Manual Installation

See the detailed [README](lynx_robot/README.md) for manual installation steps.

## First Time Setup

### 1. Hardware Connections

Connect your hardware according to this diagram:

**Motors:**
- Left Motor: GPIO 17
- Right Motor: GPIO 18
- Steering Servo: GPIO 27

**Ultrasonic Sensors (HC-SR04):**
- Front Sensor: Trig GPIO 23, Echo GPIO 24
- Back Sensor: Trig GPIO 25, Echo GPIO 8
- Left Sensor: Trig GPIO 7, Echo GPIO 1
- Right Sensor: Trig GPIO 12, Echo GPIO 16
- Operating Voltage: 5V DC
- Range: 2cm to 400cm
- Used for: Position detection AND object detection

**Camera:**
- USB Camera: Any USB port

**Power:**
- Motor Driver: 7.4V (or appropriate voltage)
- Raspberry Pi: 5V USB-C

### 2. Configure the Robot

Edit configuration files in `lynx_robot/config/`:

**Motor Configuration** (`motor_config.yaml`):
```yaml
motor_controller:
  ros__parameters:
    wheel_base: 0.3  # Distance between wheels (meters)
    max_speed: 1.0   # Maximum speed (m/s)
```

**Camera Configuration** (`camera_config.yaml`):
```yaml
camera_node:
  ros__parameters:
    camera_index: 0  # USB camera device number
    image_width: 640
    image_height: 480
```

**Ultrasonic Sensor Configuration** (`ultrasonic_config.yaml`):
```yaml
ultrasonic_sensor:
  ros__parameters:
    # GPIO pins for sensors
    front_trig_pin: 23
    front_echo_pin: 24
    # ... (see file for all pins)
    simulation_mode: false  # Set to true for testing without hardware
    
    # Object detection settings
    object_detection_enabled: true
    detection_threshold: 0.5  # Objects closer than 50cm are detected
    obstacle_warning_distance: 0.3  # Warn about objects within 30cm
```

**Waypoint Selector Configuration** (in launch file):
- `use_visual_map`: true/false - Enable visual map display
- `startup_delay`: 15.0 - Delay in seconds before robot starts (default: 15)

### 3. Test Camera

Before running the full system, test if your camera works:

```bash
# Check if camera is detected
ls /dev/video*

# Should show: /dev/video0 (or similar)

# Test camera with OpenCV
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Error')"
```

### 4. Create or Load a Map

**Option A: Use the example map**
```bash
# The example map is already included in lynx_robot/maps/
```

**Option B: Create your own map**
- Use GIMP, Paint, or similar tools
- Create a grayscale PNG/PGM image
- White = free space, Black = obstacles
- Save in `lynx_robot/maps/`
- Create a YAML file (see `example_map.yaml` as template)

**Option C: Use SLAM to create a map**
```bash
sudo apt install ros-humble-slam-toolbox
ros2 launch slam_toolbox online_async_launch.py
# Drive the robot around manually to build the map
```

## Running the Robot

### 1. Start the System

Open a terminal and run:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch lynx_robot lynx_robot.launch.py
```

You should see output from all nodes starting up.

### 2. Set Start and Goal Points

The system now features a **Visual Map-Based Start Position Selector**!

When you start the system, a graphical map will be displayed. Here's how to use it:

**Visual Mode (Default):**

1. A window will open showing your map with a grid overlay
2. Click on the map to select your robot's starting position
3. You'll be prompted to choose which coordinate is fixed:
   - Type 'x' if the X coordinate should be fixed (robot will determine Y using sensors)
   - Type 'y' if the Y coordinate should be fixed (robot will determine X using sensors)
4. The robot will use its ultrasonic sensors to determine the non-fixed coordinate
5. You can adjust the startup delay (default: 15 seconds)
6. After the countdown, the robot is ready to receive goal commands

**Text Mode (Fallback):**

If visual mode is unavailable, you'll see the text interface:

```
Commands:
  s <x> <y>  - Set start position
  g <x> <y>  - Set goal position
  p          - Plan path with current start and goal
  r          - Reset start and goal
  q          - Quit
```

Try these commands:

```bash
# Set start position at origin
s 0 0

# Set goal position 2 meters forward and 2 meters right
g 2 2

# Plan and execute the path
p
```

The robot should now:
1. Plan a path from start to goal
2. Begin moving autonomously
3. Avoid obstacles based on the map
4. Use camera for landmark detection

### 3. Monitor the Robot

Open additional terminals to monitor:

**Terminal 2 - View camera feed:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
# Select /camera/image_raw
```

**Terminal 3 - Monitor status:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /navigation_status
```

**Terminal 4 - Monitor camera detections:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /detected_objects
```

**Terminal 5 - Monitor ultrasonic object detection:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /ultrasonic_objects
```

**Terminal 6 - Monitor obstacle warnings:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /obstacle_warning
```

## Testing Individual Components

### Test Motor Controller

```bash
# In one terminal, start motor controller
ros2 run lynx_robot motor_controller

# In another terminal, send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
# Robot should move forward slowly

# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### Test Camera

```bash
ros2 run lynx_robot camera_node

# In another terminal, view images
ros2 run rqt_image_view rqt_image_view
```

### Test Ultrasonic Sensors with Object Detection

```bash
# Start ultrasonic sensor node
ros2 run lynx_robot ultrasonic_sensor

# In another terminal, monitor sensor readings
ros2 topic echo /ultrasonic_distances
# Should show [front, back, left, right] distances in meters

# Monitor object detections
ros2 topic echo /ultrasonic_objects
# Will show JSON data when objects are detected within threshold

# Monitor obstacle warnings
ros2 topic echo /obstacle_warning
# Will show 'true' when obstacles are very close (< 30cm)

# Or view JSON format sensor info
ros2 topic echo /ultrasonic_info

# Test object detection by placing objects at different distances:
# - Place object > 50cm away: No detection
# - Place object at 30-50cm: Object detected
# - Place object < 30cm: Obstacle warning activated
```

### Test Object Detection

```bash
# Start camera
ros2 run lynx_robot camera_node

# In another terminal, start detector
ros2 run lynx_robot object_detector

# View detections
ros2 topic echo /detected_objects
```

## Common Issues and Solutions

### Issue: Camera not found

**Solution:**
```bash
# Check camera connection
ls /dev/video*

# If not found, reconnect USB cable
# If found but not working, change camera_index in config
```

### Issue: Motors not responding

**Solution:**
```bash
# Check GPIO permissions
groups $USER  # Should include 'gpio'

# If not, run:
sudo usermod -a -G gpio $USER
# Then log out and back in

# Test GPIO
gpio readall  # Should show pin status
```

### Issue: "No path found"

**Solution:**
- Check that start and goal are both in free space on the map
- Try positions closer together
- Verify map loaded correctly: `ros2 topic echo /map_info`

### Issue: Robot moves erratically

**Solution:**
- Reduce max speeds in `motor_config.yaml`
- Check motor connections
- Calibrate wheel parameters (wheel_base, wheel_radius)

## Next Steps

Once your robot is running:

1. **Tune Parameters**: Adjust speeds, tolerances in config files
2. **Add Custom Maps**: Create maps of your environment
3. **Enhance Detection**: Train custom object detectors
4. **Add Features**: Extend nodes with new capabilities

## Stopping the Robot

**Emergency Stop:**
- Press `Ctrl+C` in the launch terminal
- All nodes will shut down gracefully

**Normal Stop:**
- Type `q` in the waypoint selector
- Press `Ctrl+C` to stop all nodes

## Getting Help

- Check the detailed [README](lynx_robot/README.md)
- Review ROS 2 logs: `~/.ros/log/`
- Open an issue on GitHub
- Check ROS 2 documentation

## Safety Reminders

⚠️ **Important:**
- Always test in a safe, open area first
- Keep maximum speeds conservative initially
- Be ready to press Ctrl+C for emergency stop
- Ensure motor driver is properly powered
- Keep hands clear of moving parts

Happy navigating! 🤖
