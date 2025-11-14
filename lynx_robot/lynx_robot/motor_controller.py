#!/usr/bin/env python3
"""Motor Controller Node for L.Y.N.X. Robot.

This node controls the robot's 2 motors with front-wheel steering.
It subscribes to velocity commands and publishes to motor hardware.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math


class MotorController(Node):
    """Controls 2-motor front-wheel steering system."""

    def __init__(self):
        super().__init__('motor_controller')
        
        # Declare parameters
        self.declare_parameter('wheel_base', 0.3)  # meters
        self.declare_parameter('wheel_radius', 0.05)  # meters
        self.declare_parameter('max_speed', 1.0)  # m/s
        self.declare_parameter('max_steering_angle', 0.785)  # ~45 degrees in radians
        self.declare_parameter('motor_pin_left', 17)
        self.declare_parameter('motor_pin_right', 18)
        self.declare_parameter('steering_pin', 27)
        
        # Get parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        
        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for motor commands (for hardware interface)
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            'motor_commands',
            10
        )
        
        # Initialize GPIO (placeholder - will work on Raspberry Pi)
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            self.gpio_available = True
            self.get_logger().info('GPIO initialized successfully')
        except (ImportError, RuntimeError):
            self.gpio_available = False
            self.get_logger().warn('GPIO not available - running in simulation mode')
        
        self.get_logger().info('Motor Controller initialized')

    def cmd_vel_callback(self, msg):
        """Convert Twist commands to motor velocities.
        
        Args:
            msg: Twist message with linear.x (forward speed) and angular.z (steering)
        """
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s
        
        # Limit speeds
        linear_vel = max(-self.max_speed, min(self.max_speed, linear_vel))
        
        # Calculate steering angle from angular velocity
        # For front-wheel steering: angular_vel = linear_vel * tan(steering_angle) / wheel_base
        if abs(linear_vel) > 0.01:
            steering_angle = math.atan(angular_vel * self.wheel_base / linear_vel)
        else:
            steering_angle = 0.0
        
        # Limit steering angle
        steering_angle = max(-self.max_steering_angle, 
                           min(self.max_steering_angle, steering_angle))
        
        # For front-wheel steering, both rear motors get same speed
        # Calculate wheel speeds considering steering
        left_speed = linear_vel / self.wheel_radius
        right_speed = linear_vel / self.wheel_radius
        
        # Publish motor commands
        motor_cmd = Float32MultiArray()
        motor_cmd.data = [left_speed, right_speed, steering_angle]
        self.motor_pub.publish(motor_cmd)
        
        # Control actual motors if GPIO is available
        if self.gpio_available:
            self.control_motors(left_speed, right_speed, steering_angle)
        
        self.get_logger().debug(
            f'Motor commands - Left: {left_speed:.2f}, Right: {right_speed:.2f}, '
            f'Steering: {math.degrees(steering_angle):.1f}°'
        )

    def control_motors(self, left_speed, right_speed, steering_angle):
        """Control physical motors using GPIO.
        
        Args:
            left_speed: Left motor speed (rad/s)
            right_speed: Right motor speed (rad/s)
            steering_angle: Steering angle (radians)
        """
        # This is a placeholder for actual GPIO control
        # On Raspberry Pi, you would use PWM to control motor speed
        # and servo control for steering
        pass


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
