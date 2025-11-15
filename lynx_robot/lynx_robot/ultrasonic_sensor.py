#!/usr/bin/env python3
"""Ultrasonic Sensor Node for L.Y.N.X. Robot.

This node reads ultrasonic sensors to determine robot position.
Supports 4 sensors: front, back, left, right.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import json

# Try to import RPi.GPIO, but allow simulation mode if not available
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False


class UltrasonicSensor(Node):
    """Reads ultrasonic sensors and publishes distance data."""

    def __init__(self):
        super().__init__('ultrasonic_sensor')
        
        # Declare parameters
        self.declare_parameter('front_trig_pin', 23)
        self.declare_parameter('front_echo_pin', 24)
        self.declare_parameter('back_trig_pin', 25)
        self.declare_parameter('back_echo_pin', 8)
        self.declare_parameter('left_trig_pin', 7)
        self.declare_parameter('left_echo_pin', 1)
        self.declare_parameter('right_trig_pin', 12)
        self.declare_parameter('right_echo_pin', 16)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('simulation_mode', not GPIO_AVAILABLE)
        
        # Get parameters
        self.front_trig = self.get_parameter('front_trig_pin').value
        self.front_echo = self.get_parameter('front_echo_pin').value
        self.back_trig = self.get_parameter('back_trig_pin').value
        self.back_echo = self.get_parameter('back_echo_pin').value
        self.left_trig = self.get_parameter('left_trig_pin').value
        self.left_echo = self.get_parameter('left_echo_pin').value
        self.right_trig = self.get_parameter('right_trig_pin').value
        self.right_echo = self.get_parameter('right_echo_pin').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        
        # Publishers
        self.sensor_pub = self.create_publisher(Float32MultiArray, 'ultrasonic_distances', 10)
        self.sensor_info_pub = self.create_publisher(String, 'ultrasonic_info', 10)
        
        # Initialize GPIO if available
        if not self.simulation_mode and GPIO_AVAILABLE:
            self.setup_gpio()
        else:
            self.get_logger().warn('Running in simulation mode - using dummy sensor values')
        
        # Create timer for publishing sensor readings
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_sensor_data)
        
        self.get_logger().info('Ultrasonic Sensor node initialized')
    
    def setup_gpio(self):
        """Setup GPIO pins for ultrasonic sensors."""
        try:
            GPIO.setmode(GPIO.BCM)
            
            # Setup all trigger pins as output
            GPIO.setup(self.front_trig, GPIO.OUT)
            GPIO.setup(self.back_trig, GPIO.OUT)
            GPIO.setup(self.left_trig, GPIO.OUT)
            GPIO.setup(self.right_trig, GPIO.OUT)
            
            # Setup all echo pins as input
            GPIO.setup(self.front_echo, GPIO.IN)
            GPIO.setup(self.back_echo, GPIO.IN)
            GPIO.setup(self.left_echo, GPIO.IN)
            GPIO.setup(self.right_echo, GPIO.IN)
            
            # Initialize trigger pins to low
            GPIO.output(self.front_trig, False)
            GPIO.output(self.back_trig, False)
            GPIO.output(self.left_trig, False)
            GPIO.output(self.right_trig, False)
            
            self.get_logger().info('GPIO setup complete')
        except Exception as e:
            self.get_logger().error(f'Error setting up GPIO: {str(e)}')
            self.simulation_mode = True
    
    def measure_distance(self, trig_pin, echo_pin):
        """Measure distance using ultrasonic sensor.
        
        Args:
            trig_pin: GPIO pin for trigger
            echo_pin: GPIO pin for echo
            
        Returns:
            Distance in meters (or None if error)
        """
        if self.simulation_mode:
            # Return simulated distance (2.0 meters for simulation)
            return 2.0
        
        try:
            import time
            
            # Send trigger pulse
            GPIO.output(trig_pin, True)
            time.sleep(0.00001)  # 10 microseconds
            GPIO.output(trig_pin, False)
            
            # Wait for echo
            timeout = time.time() + 0.1  # 100ms timeout
            
            # Wait for echo start
            while GPIO.input(echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return None
            
            # Wait for echo end
            while GPIO.input(echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return None
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Speed of sound / 2, in cm
            distance_m = distance / 100.0  # Convert to meters
            
            return distance_m
            
        except Exception as e:
            self.get_logger().error(f'Error measuring distance: {str(e)}')
            return None
    
    def publish_sensor_data(self):
        """Read all sensors and publish data."""
        # Read all sensors
        front_dist = self.measure_distance(self.front_trig, self.front_echo)
        back_dist = self.measure_distance(self.back_trig, self.back_echo)
        left_dist = self.measure_distance(self.left_trig, self.left_echo)
        right_dist = self.measure_distance(self.right_trig, self.right_echo)
        
        # Create message [front, back, left, right]
        msg = Float32MultiArray()
        msg.data = [
            front_dist if front_dist is not None else -1.0,
            back_dist if back_dist is not None else -1.0,
            left_dist if left_dist is not None else -1.0,
            right_dist if right_dist is not None else -1.0
        ]
        self.sensor_pub.publish(msg)
        
        # Publish sensor info as JSON
        info = {
            'front': front_dist,
            'back': back_dist,
            'left': left_dist,
            'right': right_dist
        }
        info_msg = String()
        info_msg.data = json.dumps(info)
        self.sensor_info_pub.publish(info_msg)
    
    def __del__(self):
        """Cleanup GPIO on exit."""
        if not self.simulation_mode and GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    ultrasonic_sensor = UltrasonicSensor()
    
    try:
        rclpy.spin(ultrasonic_sensor)
    except KeyboardInterrupt:
        pass
    finally:
        ultrasonic_sensor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
