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
        
        # Get GPIO pin numbers
        self.motor_pin_left = self.get_parameter('motor_pin_left').value
        self.motor_pin_right = self.get_parameter('motor_pin_right').value
        self.steering_pin = self.get_parameter('steering_pin').value
        
        # Initialize GPIO
        self.gpio_available = False
        self.pwm_left = None
        self.pwm_right = None
        self.pwm_steering = None
        
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            
            # Setup motor pins as PWM outputs
            GPIO.setup(self.motor_pin_left, GPIO.OUT)
            GPIO.setup(self.motor_pin_right, GPIO.OUT)
            GPIO.setup(self.steering_pin, GPIO.OUT)
            
            # Initialize PWM for motor speed control (1000 Hz)
            self.pwm_left = GPIO.PWM(self.motor_pin_left, 1000)
            self.pwm_right = GPIO.PWM(self.motor_pin_right, 1000)
            self.pwm_left.start(0)  # Start with 0% duty cycle (stopped)
            self.pwm_right.start(0)
            
            # Initialize PWM for servo (50 Hz for standard servos)
            self.pwm_steering = GPIO.PWM(self.steering_pin, 50)
            self.pwm_steering.start(7.5)  # Start at center position (7.5% duty cycle)
            
            self.gpio_available = True
            self.get_logger().info(
                f'GPIO initialized: Motors on pins {self.motor_pin_left}, {self.motor_pin_right}; '
                f'Servo on pin {self.steering_pin}'
            )
        except (ImportError, RuntimeError) as e:
            self.gpio_available = False
            self.get_logger().warn(f'GPIO not available - running in simulation mode: {e}')
        
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
        if not self.gpio_available or not self.pwm_left or not self.pwm_right or not self.pwm_steering:
            return
        
        # Convert angular velocity (rad/s) to duty cycle (0-100%)
        # Assuming max_speed corresponds to 100% duty cycle
        max_angular_speed = self.max_speed / self.wheel_radius
        
        # Calculate duty cycles (0-100%)
        left_duty = abs(left_speed) / max_angular_speed * 100.0
        right_duty = abs(right_speed) / max_angular_speed * 100.0
        
        # Limit duty cycles
        left_duty = max(0.0, min(100.0, left_duty))
        right_duty = max(0.0, min(100.0, right_duty))
        
        # Set motor PWM duty cycles
        try:
            self.pwm_left.ChangeDutyCycle(left_duty)
            self.pwm_right.ChangeDutyCycle(right_duty)
            
            # Convert steering angle to servo duty cycle
            # Standard servo: 1ms (5% @ 50Hz) = -90°, 1.5ms (7.5%) = 0°, 2ms (10%) = 90°
            # Map steering_angle (-max_steering_angle to +max_steering_angle) to duty cycle (5% to 10%)
            # Center at 7.5% (neutral position)
            angle_normalized = steering_angle / self.max_steering_angle  # Range: -1 to 1
            servo_duty = 7.5 + (angle_normalized * 2.5)  # Range: 5% to 10%
            servo_duty = max(5.0, min(10.0, servo_duty))  # Safety limits
            
            self.pwm_steering.ChangeDutyCycle(servo_duty)
            
        except Exception as e:
            self.get_logger().error(f'Error controlling motors: {str(e)}')
    
    def destroy_node(self):
        """Clean up GPIO resources."""
        if self.gpio_available:
            try:
                # Stop all PWM
                if self.pwm_left:
                    self.pwm_left.stop()
                if self.pwm_right:
                    self.pwm_right.stop()
                if self.pwm_steering:
                    self.pwm_steering.stop()
                # Cleanup GPIO
                self.GPIO.cleanup()
                self.get_logger().info('GPIO cleanup complete')
            except Exception as e:
                self.get_logger().error(f'Error during GPIO cleanup: {str(e)}')
        
        super().destroy_node()


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
