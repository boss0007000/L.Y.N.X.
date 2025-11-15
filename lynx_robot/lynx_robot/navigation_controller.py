#!/usr/bin/env python3
"""Navigation Controller Node for L.Y.N.X. Robot.

This node executes planned paths by sending velocity commands to the robot.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import numpy as np
import math


class NavigationController(Node):
    """Controls robot to follow planned paths."""

    def __init__(self):
        super().__init__('navigation_controller')
        
        # Declare parameters
        self.declare_parameter('max_linear_velocity', 0.5)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s
        self.declare_parameter('goal_tolerance', 0.1)  # meters
        self.declare_parameter('control_frequency', 10.0)  # Hz
        self.declare_parameter('lookahead_distance', 0.3)  # meters
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        control_freq = self.get_parameter('control_frequency').value
        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        
        # State
        self.current_pose = None
        self.current_path = None
        self.current_waypoint_index = 0
        self.is_navigating = False
        
        # Subscribe to path
        self.path_sub = self.create_subscription(
            Path,
            'planned_path',
            self.path_callback,
            10
        )
        
        # Subscribe to odometry/pose
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Alternatively, subscribe to pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'current_pose',
            self.pose_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)
        
        # Control timer
        self.timer = self.create_timer(1.0 / control_freq, self.control_loop)
        
        self.get_logger().info('Navigation Controller initialized')

    def path_callback(self, msg):
        """Receive new path to follow.
        
        Args:
            msg: Path message
        """
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        
        self.current_path = msg
        self.current_waypoint_index = 0
        self.is_navigating = True
        
        self.get_logger().info(f'Received new path with {len(msg.poses)} waypoints')
        
        status_msg = String()
        status_msg.data = 'Navigation started'
        self.status_pub.publish(status_msg)

    def odom_callback(self, msg):
        """Update current pose from odometry.
        
        Args:
            msg: Odometry message
        """
        self.current_pose = msg.pose.pose

    def pose_callback(self, msg):
        """Update current pose.
        
        Args:
            msg: PoseStamped message
        """
        self.current_pose = msg.pose

    def control_loop(self):
        """Main control loop to follow the path."""
        if not self.is_navigating or self.current_path is None or self.current_pose is None:
            # Stop the robot if not navigating
            if not self.is_navigating:
                self.stop_robot()
            return
        
        # Get current position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_theta = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # Get target waypoint
        if self.current_waypoint_index >= len(self.current_path.poses):
            # Reached the end of path
            self.stop_robot()
            self.is_navigating = False
            status_msg = String()
            status_msg.data = 'Goal reached'
            self.status_pub.publish(status_msg)
            self.get_logger().info('Goal reached')
            return
        
        target_pose = self.current_path.poses[self.current_waypoint_index]
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        
        # Calculate distance to current waypoint
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # Move to next waypoint if close enough
        if distance < self.goal_tolerance:
            self.current_waypoint_index += 1
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_index}/{len(self.current_path.poses)}'
            )
            return
        
        # Pure pursuit controller
        # Look ahead for smoother trajectory
        lookahead_waypoint = self.get_lookahead_point(current_x, current_y)
        if lookahead_waypoint is not None:
            target_x = lookahead_waypoint.pose.position.x
            target_y = lookahead_waypoint.pose.position.y
        
        # Calculate angle to target
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        angle_error = self.normalize_angle(angle_to_target - current_theta)
        
        # Calculate velocity commands
        cmd = Twist()
        
        # Linear velocity (reduced when turning)
        if abs(angle_error) > 0.5:  # If angle error > ~30 degrees
            cmd.linear.x = self.max_linear_vel * 0.3
        else:
            cmd.linear.x = self.max_linear_vel
        
        # Angular velocity (proportional control)
        kp_angular = 2.0
        cmd.angular.z = kp_angular * angle_error
        
        # Limit velocities
        cmd.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, cmd.angular.z))
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)

    def get_lookahead_point(self, current_x, current_y):
        """Get lookahead point on path.
        
        Args:
            current_x: Current x position
            current_y: Current y position
            
        Returns:
            PoseStamped of lookahead point or None
        """
        for i in range(self.current_waypoint_index, len(self.current_path.poses)):
            pose = self.current_path.poses[i]
            x = pose.pose.position.x
            y = pose.pose.position.y
            dist = math.sqrt((x - current_x)**2 + (y - current_y)**2)
            
            if dist >= self.lookahead_dist:
                return pose
        
        # Return last point if no point is beyond lookahead distance
        return self.current_path.poses[-1]

    def stop_robot(self):
        """Send zero velocity command to stop robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    @staticmethod
    def quaternion_to_yaw(quaternion):
        """Convert quaternion to yaw angle.
        
        Args:
            quaternion: Quaternion orientation
            
        Returns:
            Yaw angle in radians
        """
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi].
        
        Args:
            angle: Angle in radians
            
        Returns:
            Normalized angle
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    navigation_controller = NavigationController()
    
    try:
        rclpy.spin(navigation_controller)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
