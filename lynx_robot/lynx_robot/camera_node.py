#!/usr/bin/env python3
"""Camera Node for L.Y.N.X. Robot.

This node interfaces with USB camera and publishes images.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraNode(Node):
    """Captures and publishes images from USB camera."""

    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_name', 'lynx_camera')
        
        # Get parameters
        camera_index = self.get_parameter('camera_index').value
        frame_rate = self.get_parameter('frame_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        camera_name = self.get_parameter('camera_name').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {camera_index}')
            self.camera_available = False
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, frame_rate)
            self.camera_available = True
            self.get_logger().info(f'Camera {camera_index} opened successfully')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        
        # Timer for publishing images
        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Camera info
        self.camera_info = self.create_camera_info(camera_name)
        
        self.get_logger().info('Camera Node initialized')

    def create_camera_info(self, camera_name):
        """Create camera info message with default calibration.
        
        Args:
            camera_name: Name of the camera
            
        Returns:
            CameraInfo message
        """
        camera_info = CameraInfo()
        camera_info.header.frame_id = camera_name
        camera_info.width = self.image_width
        camera_info.height = self.image_height
        
        # Default camera matrix (should be calibrated for real use)
        fx = self.image_width  # focal length x
        fy = self.image_width  # focal length y
        cx = self.image_width / 2  # principal point x
        cy = self.image_height / 2  # principal point y
        
        camera_info.k = [fx, 0.0, cx,
                        0.0, fy, cy,
                        0.0, 0.0, 1.0]
        
        # Distortion coefficients (assuming no distortion)
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Rectification matrix (identity)
        camera_info.r = [1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0]
        
        # Projection matrix
        camera_info.p = [fx, 0.0, cx, 0.0,
                        0.0, fy, cy, 0.0,
                        0.0, 0.0, 1.0, 0.0]
        
        return camera_info

    def timer_callback(self):
        """Capture and publish camera frames."""
        if not self.camera_available:
            # Create dummy image for testing
            frame = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
            cv2.putText(frame, 'NO CAMERA', (50, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        else:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('Failed to capture frame')
                return
        
        # Convert to ROS Image message
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_frame'
            
            # Update camera info timestamp
            self.camera_info.header.stamp = image_msg.header.stamp
            
            # Publish
            self.image_pub.publish(image_msg)
            self.camera_info_pub.publish(self.camera_info)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')

    def destroy_node(self):
        """Clean up resources."""
        if self.camera_available and self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
