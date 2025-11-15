#!/usr/bin/env python3
"""Ultrasonic Sensor Node for L.Y.N.X. Robot.

This node reads ultrasonic sensors to determine robot position and detect objects.
Supports 4 HC-SR04 sensors: front, back, left, right.
Range: 2cm to 400cm (0.02m to 4.0m)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
import json
import time

# Try to import RPi.GPIO, but allow simulation mode if not available
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False


class UltrasonicSensor(Node):
    """Reads HC-SR04 ultrasonic sensors and publishes distance data with object detection."""

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
        
        # Object detection parameters
        self.declare_parameter('object_detection_enabled', True)
        self.declare_parameter('detection_threshold', 0.5)  # meters - objects closer than this are detected
        self.declare_parameter('min_detection_distance', 0.02)  # 2cm - HC-SR04 minimum range
        self.declare_parameter('max_detection_distance', 4.0)  # 400cm - HC-SR04 maximum range
        self.declare_parameter('obstacle_warning_distance', 0.3)  # 30cm - warn about close obstacles
        
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
        
        # Object detection parameters
        self.object_detection_enabled = self.get_parameter('object_detection_enabled').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.min_distance = self.get_parameter('min_detection_distance').value
        self.max_distance = self.get_parameter('max_detection_distance').value
        self.warning_distance = self.get_parameter('obstacle_warning_distance').value
        
        # Publishers
        self.sensor_pub = self.create_publisher(Float32MultiArray, 'ultrasonic_distances', 10)
        self.sensor_info_pub = self.create_publisher(String, 'ultrasonic_info', 10)
        
        # Object detection publishers
        self.object_detected_pub = self.create_publisher(String, 'ultrasonic_objects', 10)
        self.obstacle_warning_pub = self.create_publisher(Bool, 'obstacle_warning', 10)
        
        # Object detection state
        self.previous_detections = {
            'front': None,
            'back': None,
            'left': None,
            'right': None
        }
        
        # Initialize GPIO if available
        if not self.simulation_mode and GPIO_AVAILABLE:
            self.setup_gpio()
        else:
            self.get_logger().warn('Running in simulation mode - using dummy sensor values')
        
        # Create timer for publishing sensor readings
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_sensor_data)
        
        self.get_logger().info(f'Ultrasonic Sensor node initialized (HC-SR04)')
        if self.object_detection_enabled:
            self.get_logger().info(f'Object detection enabled: threshold={self.detection_threshold}m, warning={self.warning_distance}m')
    
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
        """Measure distance using HC-SR04 ultrasonic sensor.
        
        Args:
            trig_pin: GPIO pin for trigger
            echo_pin: GPIO pin for echo
            
        Returns:
            Distance in meters (or None if error or out of range)
            HC-SR04 valid range: 0.02m to 4.0m (2cm to 400cm)
        """
        if self.simulation_mode:
            # Return simulated distance (2.0 meters for simulation)
            return 2.0
        
        try:
            # Send trigger pulse (10 microsecond pulse)
            GPIO.output(trig_pin, True)
            time.sleep(0.00001)  # 10 microseconds
            GPIO.output(trig_pin, False)
            
            # Wait for echo
            timeout = time.time() + 0.1  # 100ms timeout (sufficient for 4m range)
            
            # Wait for echo start
            pulse_start = time.time()
            while GPIO.input(echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return None
            
            # Wait for echo end
            pulse_end = time.time()
            while GPIO.input(echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return None
            
            # Calculate distance
            # Speed of sound = 343 m/s at 20°C
            # Distance = (time * speed) / 2 (divide by 2 for round trip)
            pulse_duration = pulse_end - pulse_start
            distance_m = (pulse_duration * 343.0) / 2.0  # Distance in meters
            
            # Validate range (HC-SR04: 2cm to 400cm)
            if distance_m < self.min_distance or distance_m > self.max_distance:
                return None
            
            return distance_m
            
        except Exception as e:
            self.get_logger().error(f'Error measuring distance: {str(e)}')
            return None
    
    def detect_object(self, distance, direction):
        """Detect if an object is present based on distance threshold.
        
        Args:
            distance: Measured distance in meters
            direction: Sensor direction ('front', 'back', 'left', 'right')
            
        Returns:
            dict: Object detection information or None
        """
        if distance is None or not self.object_detection_enabled:
            return None
        
        # Check if object is within detection threshold
        if distance <= self.detection_threshold:
            detection = {
                'direction': direction,
                'distance': distance,
                'detected': True,
                'is_obstacle': distance <= self.warning_distance,
                'timestamp': time.time()
            }
            
            # Only log if this is a new detection or significantly different
            prev_dist = self.previous_detections[direction]
            if prev_dist is None or abs(distance - prev_dist) > 0.05:  # 5cm change
                if detection['is_obstacle']:
                    self.get_logger().warn(f'OBSTACLE DETECTED: {direction} at {distance:.2f}m')
                else:
                    self.get_logger().info(f'Object detected: {direction} at {distance:.2f}m')
            
            self.previous_detections[direction] = distance
            return detection
        else:
            self.previous_detections[direction] = None
            return None
    
    def publish_sensor_data(self):
        """Read all sensors, detect objects, and publish data."""
        # Read all sensors
        front_dist = self.measure_distance(self.front_trig, self.front_echo)
        back_dist = self.measure_distance(self.back_trig, self.back_echo)
        left_dist = self.measure_distance(self.left_trig, self.left_echo)
        right_dist = self.measure_distance(self.right_trig, self.right_echo)
        
        # Create distance message [front, back, left, right]
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
            'right': right_dist,
            'timestamp': time.time()
        }
        info_msg = String()
        info_msg.data = json.dumps(info)
        self.sensor_info_pub.publish(info_msg)
        
        # Object detection
        if self.object_detection_enabled:
            detections = []
            obstacle_warning = False
            
            # Check each sensor for object detection
            for direction, distance in [
                ('front', front_dist),
                ('back', back_dist),
                ('left', left_dist),
                ('right', right_dist)
            ]:
                detection = self.detect_object(distance, direction)
                if detection:
                    detections.append(detection)
                    if detection['is_obstacle']:
                        obstacle_warning = True
            
            # Publish detected objects
            if detections:
                objects_msg = String()
                objects_msg.data = json.dumps({
                    'detections': detections,
                    'count': len(detections),
                    'timestamp': time.time()
                })
                self.object_detected_pub.publish(objects_msg)
            
            # Publish obstacle warning
            warning_msg = Bool()
            warning_msg.data = obstacle_warning
            self.obstacle_warning_pub.publish(warning_msg)
    
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
