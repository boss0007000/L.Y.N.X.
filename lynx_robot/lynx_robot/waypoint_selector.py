#!/usr/bin/env python3
"""Waypoint Selector Node for L.Y.N.X. Robot.

This node provides an interface to select start and goal positions for navigation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import OccupancyGrid
import json
import sys
import time
import threading
import numpy as np

# Try to import matplotlib for map visualization
try:
    import matplotlib
    matplotlib.use('TkAgg')  # Use TkAgg backend for interactive display
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available. Map visualization disabled.")


class WaypointSelector(Node):
    """Interactive waypoint selection interface with visual map display."""

    def __init__(self):
        super().__init__('waypoint_selector')
        
        # Declare parameters
        self.declare_parameter('use_interactive_mode', True)
        self.declare_parameter('use_visual_map', True)
        self.declare_parameter('startup_delay', 15.0)  # Default 15 seconds delay
        
        # Get parameters
        self.interactive_mode = self.get_parameter('use_interactive_mode').value
        self.use_visual_map = self.get_parameter('use_visual_map').value and MATPLOTLIB_AVAILABLE
        self.startup_delay = self.get_parameter('startup_delay').value
        
        # Publishers
        self.planning_request_pub = self.create_publisher(String, 'planning_request', 10)
        self.start_pose_pub = self.create_publisher(PoseStamped, 'start_pose', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        self.ultrasonic_sub = self.create_subscription(
            Float32MultiArray,
            'ultrasonic_distances',
            self.ultrasonic_callback,
            10
        )
        
        # State
        self.start_position = None
        self.goal_position = None
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.map_width = 0
        self.map_height = 0
        self.ultrasonic_distances = [0.0, 0.0, 0.0, 0.0]  # [front, back, left, right]
        
        # Visual map state
        self.fig = None
        self.ax = None
        self.selected_point = None
        self.fixed_coordinate = None  # 'x' or 'y'
        self.coordinate_determined = False
        
        self.get_logger().info('Waypoint Selector initialized')
        
        if self.interactive_mode:
            self.get_logger().info('Starting interactive mode...')
            if self.use_visual_map:
                # Wait a bit for map to be published
                self.get_logger().info('Waiting for map data...')
                time.sleep(2.0)
                if self.map_data is not None:
                    self.run_visual_selection()
                else:
                    self.get_logger().warn('No map data received, falling back to text mode')
                    self.timer = self.create_timer(0.1, self.check_input)
                    self.show_menu()
            else:
                self.timer = self.create_timer(0.1, self.check_input)
                self.show_menu()
        else:
            self.get_logger().info('Non-interactive mode - waiting for programmatic input')

    def show_menu(self):
        """Display interactive menu."""
        print("\n" + "="*50)
        print("L.Y.N.X. Robot Waypoint Selector")
        print("="*50)
        print("\nCommands:")
        print("  s <x> <y>  - Set start position")
        print("  g <x> <y>  - Set goal position")
        print("  p          - Plan path with current start and goal")
        print("  r          - Reset start and goal")
        print("  q          - Quit")
        print("\nExample: s 0 0")
        print("         g 5 5")
        print("         p")
        print("="*50 + "\n")
    
    def map_callback(self, msg):
        """Receive map data.
        
        Args:
            msg: OccupancyGrid message
        """
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        
        # Convert to 2D numpy array
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        self.get_logger().info(f'Map received: {self.map_width}x{self.map_height}, resolution: {self.map_resolution}')
    
    def ultrasonic_callback(self, msg):
        """Receive ultrasonic sensor data.
        
        Args:
            msg: Float32MultiArray message [front, back, left, right]
        """
        self.ultrasonic_distances = list(msg.data)
    
    def determine_position_from_sensors(self, fixed_coord, fixed_value):
        """Determine the non-fixed coordinate using ultrasonic sensors.
        
        Args:
            fixed_coord: 'x' or 'y' - which coordinate is fixed
            fixed_value: The value of the fixed coordinate
            
        Returns:
            tuple: (x, y) position
        """
        if fixed_coord == 'x':
            # X is fixed, determine Y from left/right sensors
            # Use the average of distances from walls
            left_dist = self.ultrasonic_distances[2]
            right_dist = self.ultrasonic_distances[3]
            
            # Assume robot is between two walls
            # Y position = left_dist (distance from left wall)
            # Convert sensor distance to map coordinates
            y_position = self.map_origin[1] + left_dist / self.map_resolution
            
            self.get_logger().info(f'Determined Y position: {y_position} from left sensor: {left_dist}m')
            return (fixed_value, y_position)
        else:
            # Y is fixed, determine X from front/back sensors
            front_dist = self.ultrasonic_distances[0]
            back_dist = self.ultrasonic_distances[1]
            
            # X position = back_dist (distance from back wall)
            # Convert sensor distance to map coordinates
            x_position = self.map_origin[0] + back_dist / self.map_resolution
            
            self.get_logger().info(f'Determined X position: {x_position} from back sensor: {back_dist}m')
            return (x_position, fixed_value)
    
    def run_visual_selection(self):
        """Run visual map-based selection interface."""
        if self.map_data is None:
            self.get_logger().error('No map data available for visual selection')
            return
        
        print("\n" + "="*60)
        print("L.Y.N.X. Robot Visual Map-Based Start Position Selector")
        print("="*60)
        print("\nINSTRUCTIONS:")
        print("1. Click on the map to select the robot's starting position")
        print("2. You will then choose if X or Y coordinate is fixed")
        print("3. The robot will use ultrasonic sensors to determine the other coordinate")
        print(f"4. After {self.startup_delay} seconds delay, the robot will start")
        print("="*60 + "\n")
        
        # Create figure and axis
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_title('Click on map to select start position\n(Grid shows X,Y coordinates)', fontsize=14)
        
        # Display map
        # Convert occupancy grid to RGB for display
        # -1 (unknown) = gray, 0 (free) = white, 100 (occupied) = black
        display_map = np.zeros((self.map_height, self.map_width, 3))
        display_map[self.map_data == -1] = [0.5, 0.5, 0.5]  # Gray for unknown
        display_map[self.map_data == 0] = [1.0, 1.0, 1.0]   # White for free
        display_map[self.map_data == 100] = [0.0, 0.0, 0.0]  # Black for occupied
        
        self.ax.imshow(display_map, origin='lower', extent=[
            self.map_origin[0],
            self.map_origin[0] + self.map_width * self.map_resolution,
            self.map_origin[1],
            self.map_origin[1] + self.map_height * self.map_resolution
        ])
        
        # Add grid
        self.ax.grid(True, alpha=0.3, color='blue')
        self.ax.set_xlabel('X coordinate (meters)', fontsize=12)
        self.ax.set_ylabel('Y coordinate (meters)', fontsize=12)
        
        # Connect click event
        self.fig.canvas.mpl_connect('button_press_event', self.on_map_click)
        
        plt.tight_layout()
        plt.show()
    
    def on_map_click(self, event):
        """Handle click on map.
        
        Args:
            event: Matplotlib mouse event
        """
        if event.inaxes != self.ax:
            return
        
        x, y = event.xdata, event.ydata
        self.selected_point = (x, y)
        
        # Draw selected point
        self.ax.plot(x, y, 'ro', markersize=10, label='Selected Start')
        self.ax.legend()
        self.fig.canvas.draw()
        
        print(f"\nSelected position: X={x:.2f}, Y={y:.2f}")
        
        # Ask user which coordinate is fixed
        plt.pause(0.1)
        self.ask_fixed_coordinate()
    
    def ask_fixed_coordinate(self):
        """Ask user which coordinate is fixed."""
        print("\n" + "="*50)
        print("Which coordinate should be FIXED?")
        print("="*50)
        print("Enter 'x' if X coordinate is fixed (robot determines Y)")
        print("Enter 'y' if Y coordinate is fixed (robot determines X)")
        print("="*50)
        
        while True:
            try:
                user_input = input("\nEnter 'x' or 'y': ").strip().lower()
                if user_input in ['x', 'y']:
                    self.fixed_coordinate = user_input
                    break
                else:
                    print("Invalid input. Please enter 'x' or 'y'")
            except KeyboardInterrupt:
                print("\nExiting...")
                plt.close(self.fig)
                return
        
        # Ask about delay
        self.ask_startup_delay()
    
    def ask_startup_delay(self):
        """Ask user if they want to change the startup delay."""
        print(f"\nCurrent startup delay: {self.startup_delay} seconds")
        change_delay = input("Change delay? (y/n) [n]: ").strip().lower()
        
        if change_delay == 'y':
            try:
                new_delay = float(input("Enter new delay in seconds: "))
                if new_delay >= 0:
                    self.startup_delay = new_delay
                    print(f"Delay set to {self.startup_delay} seconds")
                else:
                    print("Invalid delay, keeping current value")
            except ValueError:
                print("Invalid input, keeping current value")
        
        # Start the positioning process
        self.start_positioning_sequence()
    
    def start_positioning_sequence(self):
        """Start the positioning sequence with sensor reading and delay."""
        x, y = self.selected_point
        
        print("\n" + "="*60)
        print("POSITIONING SEQUENCE STARTED")
        print("="*60)
        print(f"Selected position: ({x:.2f}, {y:.2f})")
        print(f"Fixed coordinate: {self.fixed_coordinate.upper()}")
        
        if self.fixed_coordinate == 'x':
            print(f"  X = {x:.2f} (FIXED)")
            print(f"  Y = Will be determined by ultrasonic sensors")
        else:
            print(f"  X = Will be determined by ultrasonic sensors")
            print(f"  Y = {y:.2f} (FIXED)")
        
        print(f"\nReading ultrasonic sensors...")
        
        # Wait for sensor readings
        rclpy.spin_once(self, timeout_sec=1.0)
        
        # Determine actual position using sensors
        if self.fixed_coordinate == 'x':
            actual_position = self.determine_position_from_sensors('x', x)
        else:
            actual_position = self.determine_position_from_sensors('y', y)
        
        print(f"\nDetermined position: ({actual_position[0]:.2f}, {actual_position[1]:.2f})")
        print(f"  X = {actual_position[0]:.2f}")
        print(f"  Y = {actual_position[1]:.2f}")
        
        # Update the display
        if self.fig and self.ax:
            self.ax.plot(actual_position[0], actual_position[1], 'g*', markersize=15, 
                        label='Actual Start (with sensors)')
            self.ax.legend()
            self.fig.canvas.draw()
        
        # Set start position
        self.set_start_position(actual_position[0], actual_position[1])
        
        # Start countdown
        print("\n" + "="*60)
        print(f"Starting in {self.startup_delay} seconds...")
        print("="*60)
        
        for i in range(int(self.startup_delay), 0, -1):
            print(f"{i}... ", end='', flush=True)
            time.sleep(1.0)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        print("\nSTARTING!")
        print("\nNow you can set a goal position and plan a path.")
        print("Close the map window to continue to text mode or use the visual interface.")
        
        # Allow goal selection
        plt.pause(0.1)

    def check_input(self):
        """Check for user input (non-blocking)."""
        # This is a simple implementation - in production, use proper async input
        pass

    def set_start_position(self, x, y):
        """Set start position.
        
        Args:
            x: X coordinate
            y: Y coordinate
        """
        self.start_position = [float(x), float(y)]
        self.get_logger().info(f'Start position set to: {self.start_position}')
        
        # Publish as PoseStamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        self.start_pose_pub.publish(pose)

    def set_goal_position(self, x, y):
        """Set goal position.
        
        Args:
            x: X coordinate
            y: Y coordinate
        """
        self.goal_position = [float(x), float(y)]
        self.get_logger().info(f'Goal position set to: {self.goal_position}')
        
        # Publish as PoseStamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        self.goal_pose_pub.publish(pose)

    def request_path_planning(self):
        """Request path planning with current start and goal."""
        if self.start_position is None or self.goal_position is None:
            self.get_logger().error('Start and/or goal position not set')
            print("ERROR: Please set both start and goal positions first")
            return False
        
        # Create planning request
        request = {
            'start': self.start_position,
            'goal': self.goal_position
        }
        
        msg = String()
        msg.data = json.dumps(request)
        self.planning_request_pub.publish(msg)
        
        self.get_logger().info(f'Path planning requested: {request}')
        print(f"Path planning requested from {self.start_position} to {self.goal_position}")
        return True

    def reset(self):
        """Reset start and goal positions."""
        self.start_position = None
        self.goal_position = None
        self.get_logger().info('Start and goal positions reset')
        print("Start and goal positions reset")

    def run_interactive(self):
        """Run interactive command loop."""
        while rclpy.ok():
            try:
                # Get user input
                user_input = input("\nEnter command: ").strip().split()
                
                if not user_input:
                    continue
                
                command = user_input[0].lower()
                
                if command == 'q':
                    print("Exiting...")
                    break
                elif command == 's':
                    if len(user_input) != 3:
                        print("Usage: s <x> <y>")
                        continue
                    try:
                        x, y = float(user_input[1]), float(user_input[2])
                        self.set_start_position(x, y)
                    except ValueError:
                        print("Invalid coordinates")
                elif command == 'g':
                    if len(user_input) != 3:
                        print("Usage: g <x> <y>")
                        continue
                    try:
                        x, y = float(user_input[1]), float(user_input[2])
                        self.set_goal_position(x, y)
                    except ValueError:
                        print("Invalid coordinates")
                elif command == 'p':
                    self.request_path_planning()
                elif command == 'r':
                    self.reset()
                else:
                    print(f"Unknown command: {command}")
                    print("Type 's', 'g', 'p', 'r', or 'q'")
                
                # Spin once to process callbacks
                rclpy.spin_once(self, timeout_sec=0.1)
                
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                self.get_logger().error(f'Error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    waypoint_selector = WaypointSelector()
    
    try:
        if waypoint_selector.interactive_mode:
            waypoint_selector.run_interactive()
        else:
            rclpy.spin(waypoint_selector)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_selector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
