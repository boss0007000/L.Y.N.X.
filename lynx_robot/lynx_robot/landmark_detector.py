#!/usr/bin/env python3
"""Landmark Detection Node for L.Y.N.X. Robot.

This node detects landmarks (like ArUco markers, QR codes, etc.) in camera images.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class LandmarkDetector(Node):
    """Detects landmarks in camera images."""

    def __init__(self):
        super().__init__('landmark_detector')
        
        # Declare parameters
        self.declare_parameter('marker_type', 'aruco')  # aruco, qr, or corner
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('publish_debug_image', True)
        
        # Get parameters
        self.marker_type = self.get_parameter('marker_type').value
        aruco_dict_name = self.get_parameter('aruco_dict').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize ArUco detector if using ArUco markers
        if self.marker_type == 'aruco':
            try:
                aruco_dict = getattr(cv2.aruco, aruco_dict_name)
                self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict)
                self.aruco_parameters = cv2.aruco.DetectorParameters()
                self.aruco_detector = cv2.aruco.ArucoDetector(
                    self.aruco_dictionary, 
                    self.aruco_parameters
                )
                self.get_logger().info(f'ArUco detector initialized with {aruco_dict_name}')
            except AttributeError:
                # Fallback for older OpenCV versions
                self.aruco_dictionary = cv2.aruco.Dictionary_get(
                    getattr(cv2.aruco, aruco_dict_name)
                )
                self.aruco_parameters = cv2.aruco.DetectorParameters_create()
                self.aruco_detector = None
                self.get_logger().info('Using legacy ArUco API')
        
        # Initialize QR code detector
        if self.marker_type == 'qr':
            self.qr_detector = cv2.QRCodeDetector()
        
        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.landmark_pub = self.create_publisher(String, 'detected_landmarks', 10)
        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(Image, 'landmark/debug_image', 10)
        
        self.get_logger().info(f'Landmark Detector initialized with {self.marker_type} detection')

    def image_callback(self, msg):
        """Process incoming images and detect landmarks.
        
        Args:
            msg: Image message from camera
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect landmarks based on selected method
            if self.marker_type == 'aruco':
                landmarks = self.detect_aruco_markers(cv_image)
            elif self.marker_type == 'qr':
                landmarks = self.detect_qr_codes(cv_image)
            elif self.marker_type == 'corner':
                landmarks = self.detect_corners(cv_image)
            else:
                landmarks = []
            
            # Publish landmarks
            if landmarks:
                landmark_msg = String()
                landmark_msg.data = json.dumps(landmarks)
                self.landmark_pub.publish(landmark_msg)
                
                self.get_logger().info(f'Detected {len(landmarks)} landmarks')
            
            # Publish debug image if enabled
            if self.publish_debug:
                debug_image = self.draw_landmarks(cv_image.copy(), landmarks)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def detect_aruco_markers(self, image):
        """Detect ArUco markers in image.
        
        Args:
            image: OpenCV image (BGR format)
            
        Returns:
            List of detected landmarks
        """
        landmarks = []
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        try:
            # Detect markers using new API
            if self.aruco_detector is not None:
                corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            else:
                # Legacy API
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray, 
                    self.aruco_dictionary, 
                    parameters=self.aruco_parameters
                )
            
            if ids is not None:
                for i, marker_id in enumerate(ids):
                    corner = corners[i][0]
                    # Calculate center of marker
                    center_x = int(np.mean(corner[:, 0]))
                    center_y = int(np.mean(corner[:, 1]))
                    
                    landmark = {
                        'type': 'aruco',
                        'id': int(marker_id[0]),
                        'center': [center_x, center_y],
                        'corners': corner.tolist()
                    }
                    landmarks.append(landmark)
        
        except Exception as e:
            self.get_logger().error(f'ArUco detection error: {str(e)}')
        
        return landmarks

    def detect_qr_codes(self, image):
        """Detect QR codes in image.
        
        Args:
            image: OpenCV image (BGR format)
            
        Returns:
            List of detected landmarks
        """
        landmarks = []
        
        try:
            data, bbox, _ = self.qr_detector.detectAndDecode(image)
            
            if bbox is not None and data:
                bbox = bbox[0].astype(int)
                # Calculate center
                center_x = int(np.mean(bbox[:, 0]))
                center_y = int(np.mean(bbox[:, 1]))
                
                landmark = {
                    'type': 'qr',
                    'data': data,
                    'center': [center_x, center_y],
                    'corners': bbox.tolist()
                }
                landmarks.append(landmark)
        
        except Exception as e:
            self.get_logger().error(f'QR detection error: {str(e)}')
        
        return landmarks

    def detect_corners(self, image):
        """Detect corners (like Harris corners) in image.
        
        Args:
            image: OpenCV image (BGR format)
            
        Returns:
            List of detected landmarks
        """
        landmarks = []
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Harris corner detection
        corners = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.01, minDistance=10)
        
        if corners is not None:
            for corner in corners:
                x, y = corner.ravel()
                landmark = {
                    'type': 'corner',
                    'center': [int(x), int(y)]
                }
                landmarks.append(landmark)
        
        return landmarks

    def draw_landmarks(self, image, landmarks):
        """Draw detected landmarks on image.
        
        Args:
            image: OpenCV image
            landmarks: List of detected landmarks
            
        Returns:
            Image with drawn landmarks
        """
        for landmark in landmarks:
            center = landmark['center']
            
            # Draw center point
            cv2.circle(image, tuple(center), 5, (0, 255, 0), -1)
            
            # Draw corners if available
            if 'corners' in landmark:
                corners = np.array(landmark['corners'], dtype=np.int32)
                cv2.polylines(image, [corners], True, (0, 255, 0), 2)
            
            # Add label
            label = f"{landmark['type']}"
            if 'id' in landmark:
                label += f" {landmark['id']}"
            elif 'data' in landmark:
                label += f" {landmark['data'][:10]}"
            
            cv2.putText(image, label, (center[0] - 20, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return image


def main(args=None):
    rclpy.init(args=args)
    landmark_detector = LandmarkDetector()
    
    try:
        rclpy.spin(landmark_detector)
    except KeyboardInterrupt:
        pass
    finally:
        landmark_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
