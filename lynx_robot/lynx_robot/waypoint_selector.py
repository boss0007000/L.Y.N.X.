#!/usr/bin/env python3
"""Waypoint Selector Node for L.Y.N.X. Robot.

This node provides an interface to select start and goal positions for navigation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import sys


class WaypointSelector(Node):
    """Interactive waypoint selection interface."""

    def __init__(self):
        super().__init__('waypoint_selector')
        
        # Declare parameters
        self.declare_parameter('use_interactive_mode', True)
        
        # Get parameters
        self.interactive_mode = self.get_parameter('use_interactive_mode').value
        
        # Publishers
        self.planning_request_pub = self.create_publisher(String, 'planning_request', 10)
        self.start_pose_pub = self.create_publisher(PoseStamped, 'start_pose', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        # State
        self.start_position = None
        self.goal_position = None
        
        self.get_logger().info('Waypoint Selector initialized')
        
        if self.interactive_mode:
            self.get_logger().info('Starting interactive mode...')
            # Run interactive mode in a separate thread to not block ROS
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
