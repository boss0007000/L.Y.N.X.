#!/usr/bin/env python3
"""Object Detection Node for L.Y.N.X. Robot.

This node performs real-time object detection using OpenCV.
Supports various detection methods including color-based and contour detection.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class ObjectDetector(Node):
    """Detects objects in camera images."""

    def __init__(self):
        super().__init__('object_detector')
        
        # Declare parameters
        self.declare_parameter('detection_method', 'color')  # color, contour, or cascade
        self.declare_parameter('min_object_area', 1000)
        self.declare_parameter('publish_debug_image', True)
        
        # Get parameters
        self.detection_method = self.get_parameter('detection_method').value
        self.min_object_area = self.get_parameter('min_object_area').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(String, 'detected_objects', 10)
        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(Image, 'detection/debug_image', 10)
        
        # Define color ranges for detection (HSV format)
        self.color_ranges = {
            'red': [(0, 100, 100), (10, 255, 255)],
            'green': [(40, 40, 40), (80, 255, 255)],
            'blue': [(100, 100, 100), (130, 255, 255)],
            'yellow': [(20, 100, 100), (40, 255, 255)]
        }
        
        self.get_logger().info(f'Object Detector initialized with {self.detection_method} method')

    def image_callback(self, msg):
        """Process incoming images and detect objects.
        
        Args:
            msg: Image message from camera
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect objects based on selected method
            if self.detection_method == 'color':
                detections = self.detect_colored_objects(cv_image)
            elif self.detection_method == 'contour':
                detections = self.detect_contours(cv_image)
            else:
                detections = []
            
            # Publish detections
            if detections:
                detection_msg = String()
                detection_msg.data = json.dumps(detections)
                self.detection_pub.publish(detection_msg)
                
                self.get_logger().info(f'Detected {len(detections)} objects')
            
            # Publish debug image if enabled
            if self.publish_debug:
                debug_image = self.draw_detections(cv_image.copy(), detections)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def detect_colored_objects(self, image):
        """Detect objects based on color.
        
        Args:
            image: OpenCV image (BGR format)
            
        Returns:
            List of detected objects with position and color
        """
        detections = []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        for color_name, (lower, upper) in self.color_ranges.items():
            # Create mask for color
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
            # Apply morphological operations to reduce noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.min_object_area:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    detection = {
                        'type': 'colored_object',
                        'color': color_name,
                        'center': [int(center_x), int(center_y)],
                        'bbox': [int(x), int(y), int(w), int(h)],
                        'area': float(area)
                    }
                    detections.append(detection)
        
        return detections

    def detect_contours(self, image):
        """Detect objects based on contours.
        
        Args:
            image: OpenCV image (BGR format)
            
        Returns:
            List of detected objects
        """
        detections = []
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_object_area:
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                
                detection = {
                    'type': 'contour_object',
                    'center': [int(center_x), int(center_y)],
                    'bbox': [int(x), int(y), int(w), int(h)],
                    'area': float(area)
                }
                detections.append(detection)
        
        return detections

    def draw_detections(self, image, detections):
        """Draw detection results on image.
        
        Args:
            image: OpenCV image
            detections: List of detected objects
            
        Returns:
            Image with drawn detections
        """
        for detection in detections:
            x, y, w, h = detection['bbox']
            center = detection['center']
            
            # Draw bounding box
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(image, tuple(center), 5, (0, 0, 255), -1)
            
            # Add label
            label = detection.get('color', detection['type'])
            cv2.putText(image, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return image


def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    
    try:
        rclpy.spin(object_detector)
    except KeyboardInterrupt:
        pass
    finally:
        object_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
