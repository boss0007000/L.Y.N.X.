#!/bin/bash
# Installation script for L.Y.N.X. Robot on Raspberry Pi
# Run with: bash install.sh

set -e

echo "========================================"
echo "L.Y.N.X. Robot Installation Script"
echo "========================================"
echo ""

# Check if running on Raspberry Pi
if [ ! -f /proc/device-tree/model ]; then
    echo "Warning: This script is designed for Raspberry Pi"
    echo "Some features may not work on other systems"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "Step 1: Updating system packages..."
sudo apt update

echo ""
echo "Step 2: Installing Python dependencies..."
sudo apt install -y python3-pip python3-opencv python3-numpy python3-yaml python3-pil

echo ""
echo "Step 3: Installing ROS 2 dependencies..."
# Check if ROS 2 is installed
if [ -z "$ROS_DISTRO" ]; then
    echo "Warning: ROS 2 not detected. Please install ROS 2 first."
    echo "See: https://docs.ros.org/en/humble/Installation.html"
else
    echo "ROS 2 $ROS_DISTRO detected"
    sudo apt install -y ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport
fi

echo ""
echo "Step 4: Installing GPIO support (Raspberry Pi)..."
if grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    sudo apt install -y python3-rpi.gpio
    # Add user to gpio group
    sudo usermod -a -G gpio $USER
    echo "Added $USER to gpio group. Please log out and back in for changes to take effect."
else
    echo "Not on Raspberry Pi, skipping GPIO installation"
fi

echo ""
echo "Step 5: Installing camera support..."
sudo apt install -y v4l-utils

echo ""
echo "Step 6: Creating ROS 2 workspace..."
WORKSPACE_DIR=~/ros2_ws
if [ ! -d "$WORKSPACE_DIR" ]; then
    mkdir -p $WORKSPACE_DIR/src
    echo "Created workspace at $WORKSPACE_DIR"
else
    echo "Workspace already exists at $WORKSPACE_DIR"
fi

echo ""
echo "Step 7: Copying package to workspace..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_DIR="$SCRIPT_DIR/lynx_robot"
if [ -d "$PACKAGE_DIR" ]; then
    cp -r "$PACKAGE_DIR" $WORKSPACE_DIR/src/
    echo "Package copied to $WORKSPACE_DIR/src/"
else
    echo "Warning: lynx_robot package not found at $PACKAGE_DIR"
fi

echo ""
echo "Step 8: Building ROS 2 package..."
cd $WORKSPACE_DIR
if [ ! -z "$ROS_DISTRO" ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --packages-select lynx_robot
    echo "Package built successfully"
else
    echo "Skipping build - ROS 2 not detected"
fi

echo ""
echo "========================================"
echo "Installation Complete!"
echo "========================================"
echo ""
echo "Next steps:"
echo "1. Log out and log back in (for GPIO permissions)"
echo "2. Source the workspace: source ~/ros2_ws/install/setup.bash"
echo "3. Launch the robot: ros2 launch lynx_robot lynx_robot.launch.py"
echo ""
echo "For more information, see lynx_robot/README.md"
echo ""
