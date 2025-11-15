# Troubleshooting Guide for L.Y.N.X. Robot

This guide helps you diagnose and fix common issues with the L.Y.N.X. robot system.

## Quick Diagnostics

Run these commands to quickly check system status:

```bash
# Check ROS 2 installation
echo $ROS_DISTRO
# Should output: humble (or your ROS 2 version)

# List running nodes
ros2 node list

# Check topics
ros2 topic list

# Test communication
ros2 topic hz /camera/image_raw  # Should show ~30 Hz
```

## Common Issues and Solutions

### 1. Launch File Issues

#### Problem: "Package 'lynx_robot' not found"

**Symptoms:**
```
Package 'lynx_robot' not found
```

**Solution:**
```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# If that doesn't work, rebuild
cd ~/ros2_ws
colcon build --packages-select lynx_robot
source install/setup.bash

# Add to bashrc for permanent fix
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

#### Problem: Launch file fails with import errors

**Symptoms:**
```
ImportError: No module named 'cv_bridge'
```

**Solution:**
```bash
# Install missing dependencies
sudo apt install ros-$ROS_DISTRO-cv-bridge
sudo apt install ros-$ROS_DISTRO-image-transport

# Or install all ROS 2 dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Camera Issues

#### Problem: "Failed to open camera 0"

**Symptoms:**
- Camera node shows error
- No images published
- Black screen in image viewer

**Diagnostic:**
```bash
# Check if camera is detected
ls /dev/video*
# Should show: /dev/video0 or similar

# List USB devices
lsusb
# Should show your camera

# Check camera capabilities
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

**Solutions:**

1. **Try different camera index:**
```bash
# Edit config/camera_config.yaml
camera_index: 1  # Try 0, 1, 2, etc.
```

2. **Check permissions:**
```bash
# Add user to video group
sudo usermod -a -G video $USER
# Log out and back in
```

3. **Test camera directly:**
```bash
# Install ffmpeg
sudo apt install ffmpeg

# Test capture
ffplay /dev/video0
```

4. **Camera not working at all:**
```bash
# Try different USB port
# Check camera LED (should light up)
# Test on another computer
# Check dmesg for errors
dmesg | grep video
```

#### Problem: Low frame rate or laggy video

**Solutions:**
```bash
# Reduce resolution in camera_config.yaml
image_width: 320
image_height: 240
frame_rate: 15

# Disable debug images in detection configs
publish_debug_image: false
```

### 3. Motor Control Issues

#### Problem: Motors don't respond

**Diagnostic:**
```bash
# Check if motor controller is running
ros2 node list | grep motor_controller

# Test by publishing directly
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Check motor commands being published
ros2 topic echo /motor_commands
```

**Solutions:**

1. **GPIO permission error:**
```bash
# Check current permissions
groups $USER
# Should include 'gpio'

# If not, add to group
sudo usermod -a -G gpio $USER
# Log out and back in

# Test GPIO
gpio readall  # Install with: sudo apt install wiringpi
```

2. **Wrong GPIO pins:**
```bash
# Edit motor_config.yaml
motor_pin_left: 17  # Verify these match your wiring
motor_pin_right: 18
steering_pin: 27
```

3. **Motor driver power issue:**
```bash
# Check power supply with multimeter
# Verify motor driver has power LED on
# Test motors directly with motor driver
# Check common ground connection
```

4. **RPi.GPIO not installed:**
```bash
sudo apt install python3-rpi.gpio
# Or
pip3 install RPi.GPIO
```

#### Problem: Motors spin in wrong direction

**Solution:**
```bash
# Physically swap motor wires at motor driver
# Or modify motor_controller.py to invert direction
```

#### Problem: Servo doesn't respond

**Solutions:**
```bash
# Check servo power (needs 5V, sufficient current)
# Test servo with simple script:
python3 << EOF
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.OUT)
pwm = GPIO.PWM(27, 50)
pwm.start(7.5)
time.sleep(2)
pwm.stop()
GPIO.cleanup()
EOF
```

### 4. Navigation Issues

#### Problem: "No path found"

**Diagnostic:**
```bash
# Check if map is loaded
ros2 topic echo /map_info --once

# Check start and goal positions
ros2 topic echo /start_pose --once
ros2 topic echo /goal_pose --once

# View planning status
ros2 topic echo /planning_status
```

**Solutions:**

1. **Start/goal in occupied space:**
```bash
# Verify positions are in free space (white on map)
# Use smaller coordinates
# Check map resolution and origin
```

2. **Map not loaded:**
```bash
# Verify map file exists
ls ~/ros2_ws/install/lynx_robot/share/lynx_robot/maps/

# Check map_parser output
ros2 node info /map_parser

# Manually specify map
ros2 param set /map_parser map_file /path/to/map.yaml
```

3. **Increase goal tolerance:**
```bash
# Edit planner_config.yaml
goal_tolerance: 0.5  # Increase from 0.2
```

#### Problem: Robot doesn't move after path planning

**Diagnostic:**
```bash
# Check if path is published
ros2 topic echo /planned_path

# Check navigation status
ros2 topic echo /navigation_status

# Check if velocity commands are sent
ros2 topic echo /cmd_vel
```

**Solutions:**

1. **No odometry/pose:**
```bash
# Navigation controller needs pose feedback
# Publish dummy pose for testing:
ros2 topic pub /current_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" -r 10

# Or implement odometry from encoders
```

2. **Path not being followed:**
```bash
# Check control frequency
ros2 param get /navigation_controller control_frequency

# Reduce speeds
ros2 param set /navigation_controller max_linear_velocity 0.2
```

#### Problem: Robot moves erratically

**Solutions:**
```bash
# Reduce speeds in navigation_config.yaml
max_linear_velocity: 0.3  # Reduce from 0.5
max_angular_velocity: 0.5  # Reduce from 1.0

# Tune control parameters
lookahead_distance: 0.5  # Increase for smoother paths

# Calibrate wheel parameters
wheel_base: 0.30  # Measure actual distance
wheel_radius: 0.05  # Measure actual radius
```

### 5. Detection Issues

#### Problem: No objects detected

**Diagnostic:**
```bash
# Check if detector is running
ros2 node list | grep detector

# Check detections
ros2 topic echo /detected_objects

# View debug image
ros2 run rqt_image_view rqt_image_view
# Select /detection/debug_image
```

**Solutions:**

1. **Adjust detection parameters:**
```bash
# Edit detection_config.yaml
min_object_area: 500  # Lower threshold
detection_method: "contour"  # Try different method
```

2. **Color detection not working:**
```python
# Colors might be off - adjust HSV ranges in object_detector.py
# Example for red:
'red': [(0, 100, 100), (10, 255, 255)]  # Adjust these values
```

#### Problem: Landmark detection not working

**Solutions:**

1. **ArUco markers:**
```bash
# Ensure correct dictionary
aruco_dict: "DICT_4X4_50"  # Match your markers

# Print test markers:
# https://chev.me/arucogen/

# Check OpenCV version
python3 -c "import cv2; print(cv2.__version__)"
# Should be 4.5.0 or later
```

2. **QR codes:**
```bash
# Ensure good lighting
# Keep QR code flat and unobstructed
# Increase camera resolution
```

### 6. Performance Issues

#### Problem: High CPU usage on Raspberry Pi

**Solutions:**
```bash
# Reduce camera resolution
image_width: 320
image_height: 240
frame_rate: 15

# Disable debug images
publish_debug_image: false

# Reduce control frequency
control_frequency: 5.0  # Down from 10.0

# Use lighter detection
detection_method: "color"  # Faster than contour

# Monitor CPU
htop
```

#### Problem: Lag or delay in response

**Solutions:**
```bash
# Check system load
top

# Reduce processing
# Kill unnecessary processes

# Use lighter ROS 2 executors
# Increase QoS queue sizes
```

### 7. ROS 2 Communication Issues

#### Problem: Topics not publishing/receiving

**Diagnostic:**
```bash
# Check topic info
ros2 topic info /camera/image_raw

# Check nodes
ros2 node info /camera_node

# List all topics with types
ros2 topic list -t

# Check network (for multi-machine setups)
ros2 daemon stop
ros2 daemon start
```

**Solutions:**
```bash
# Source workspace in every terminal
source ~/ros2_ws/install/setup.bash

# Check ROS_DOMAIN_ID (for multi-robot)
echo $ROS_DOMAIN_ID

# Increase QoS queue sizes
# Edit nodes to use larger queue_size parameter
```

### 8. Installation Issues

#### Problem: colcon build fails

**Symptoms:**
```
SetuptoolsDeprecationWarning
Package 'lynx_robot' not found
```

**Solutions:**
```bash
# Update setuptools
pip3 install --upgrade setuptools

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build install log
colcon build --packages-select lynx_robot

# Check for typos in setup.py and package.xml
```

#### Problem: Python module import errors

**Solutions:**
```bash
# Install Python packages
pip3 install opencv-python numpy PyYAML Pillow

# Or use apt
sudo apt install python3-opencv python3-numpy python3-yaml python3-pil

# Check Python path
echo $PYTHONPATH
```

### 9. Map Issues

#### Problem: Map not loading

**Diagnostic:**
```bash
# Check map file exists
ls -la lynx_robot/maps/example_map.yaml

# Check map_parser logs
ros2 node info /map_parser
```

**Solutions:**
```bash
# Verify YAML syntax
cat lynx_robot/maps/example_map.yaml

# Check image path in YAML
image: example_map.pgm  # Should be relative to YAML file

# Test with absolute path
map_file: /home/user/ros2_ws/install/lynx_robot/share/lynx_robot/maps/example_map.yaml
```

#### Problem: Map appears wrong

**Solutions:**
```bash
# Check resolution
resolution: 0.05  # meters per pixel

# Check origin
origin: [0.0, 0.0, 0.0]  # x, y, theta

# View map in RViz
ros2 run rviz2 rviz2
# Add Map display, topic /map
```

## Advanced Debugging

### Enable Debug Logging

```bash
# Set log level for specific node
ros2 run lynx_robot camera_node --ros-args --log-level debug

# Or in launch file:
ros2 launch lynx_robot lynx_robot.launch.py --log-level debug
```

### Record and Replay Data

```bash
# Record topics for later analysis
ros2 bag record -a -o my_recording

# Replay
ros2 bag play my_recording
```

### Visualize with RViz2

```bash
ros2 run rviz2 rviz2

# Add displays:
# - Map (/map)
# - Path (/planned_path)
# - Image (/camera/image_raw)
# - TF (if using transforms)
```

### Check System Resources

```bash
# CPU usage
htop

# Memory
free -h

# Disk space
df -h

# Temperature (Raspberry Pi)
vcgencmd measure_temp

# GPIO status
gpio readall
```

## Getting More Help

### Check Logs

```bash
# ROS 2 logs
ls ~/.ros/log/

# System logs
journalctl -f

# Kernel messages
dmesg | tail
```

### Enable Verbose Output

```bash
# Run nodes with verbose output
ros2 run lynx_robot camera_node --ros-args --log-level debug -r __node:=camera_node_debug
```

### Community Resources

- ROS 2 Documentation: https://docs.ros.org
- ROS Answers: https://answers.ros.org
- GitHub Issues: Open an issue on the repository
- Raspberry Pi Forums: https://forums.raspberrypi.com

## Quick Fix Checklist

When something isn't working, try these in order:

1. [ ] Source the workspace: `source ~/ros2_ws/install/setup.bash`
2. [ ] Check ROS 2 is running: `ros2 node list`
3. [ ] Verify hardware connections
4. [ ] Check permissions (GPIO, video, etc.)
5. [ ] Review configuration files
6. [ ] Check logs for errors
7. [ ] Test components individually
8. [ ] Restart the entire system
9. [ ] Rebuild the package: `colcon build --packages-select lynx_robot`
10. [ ] Check this guide for specific issue

## Emergency Procedures

### Emergency Stop

```bash
# Press Ctrl+C in launch terminal
# Or publish zero velocity
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### Reset System

```bash
# Kill all ROS nodes
killall -9 python3

# Restart
ros2 launch lynx_robot lynx_robot.launch.py
```

### Hardware Reset

```bash
# GPIO cleanup
python3 -c "import RPi.GPIO as GPIO; GPIO.cleanup()"

# Power cycle
# Disconnect battery, wait 10 seconds, reconnect
```

## Preventing Issues

### Best Practices

1. **Always test in safe environment** before real operation
2. **Start with low speeds** and gradually increase
3. **Keep backups** of working configurations
4. **Document changes** you make
5. **Test after each change** before making more changes
6. **Monitor system resources** regularly
7. **Keep software updated** but test updates
8. **Check connections** before each run

### Maintenance Schedule

**Daily (when in use):**
- Check battery voltage
- Inspect wires and connections
- Clean camera lens

**Weekly:**
- Update software packages
- Check motor mounting
- Test emergency stop

**Monthly:**
- Deep clean robot
- Check for loose screws
- Calibrate sensors
- Backup configurations

---

If you've tried everything and still have issues:
1. Document exactly what happens
2. Collect relevant logs
3. Note your hardware setup
4. Open an issue on GitHub with all details

Good luck! 🔧🤖
