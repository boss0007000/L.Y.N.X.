#!/usr/bin/env python3
"""Map Parser Node for L.Y.N.X. Robot.

This node loads and parses map files for navigation.
Supports grid-based occupancy maps.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import yaml
import numpy as np
from PIL import Image
import os


class MapParser(Node):
    """Parses and publishes map data."""

    def __init__(self):
        super().__init__('map_parser')
        
        # Declare parameters
        self.declare_parameter('map_file', '')
        self.declare_parameter('map_resolution', 0.05)  # meters per pixel
        self.declare_parameter('map_origin_x', 0.0)
        self.declare_parameter('map_origin_y', 0.0)
        self.declare_parameter('map_origin_theta', 0.0)
        self.declare_parameter('occupied_threshold', 0.65)
        self.declare_parameter('free_threshold', 0.196)
        
        # Get parameters
        self.map_file = self.get_parameter('map_file').value
        self.resolution = self.get_parameter('map_resolution').value
        self.origin_x = self.get_parameter('map_origin_x').value
        self.origin_y = self.get_parameter('map_origin_y').value
        self.origin_theta = self.get_parameter('map_origin_theta').value
        self.occupied_thresh = self.get_parameter('occupied_threshold').value
        self.free_thresh = self.get_parameter('free_threshold').value
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.map_info_pub = self.create_publisher(String, 'map_info', 10)
        
        # Map data
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        
        # Timer to publish map periodically
        self.timer = self.create_timer(1.0, self.publish_map)
        
        # Load map if file is provided
        if self.map_file:
            self.load_map(self.map_file)
        else:
            self.get_logger().warn('No map file specified, using empty map')
            self.create_empty_map(100, 100)
        
        self.get_logger().info('Map Parser initialized')

    def load_map(self, map_file):
        """Load map from file.
        
        Args:
            map_file: Path to map file (YAML or image)
        """
        try:
            if not os.path.exists(map_file):
                self.get_logger().error(f'Map file not found: {map_file}')
                self.create_empty_map(100, 100)
                return
            
            # Check file extension
            if map_file.endswith('.yaml') or map_file.endswith('.yml'):
                self.load_yaml_map(map_file)
            elif map_file.endswith(('.png', '.pgm', '.jpg', '.jpeg')):
                self.load_image_map(map_file)
            else:
                self.get_logger().error(f'Unsupported map file format: {map_file}')
                self.create_empty_map(100, 100)
                
        except Exception as e:
            self.get_logger().error(f'Error loading map: {str(e)}')
            self.create_empty_map(100, 100)

    def load_yaml_map(self, yaml_file):
        """Load map from YAML file (ROS map format).
        
        Args:
            yaml_file: Path to YAML map file
        """
        with open(yaml_file, 'r') as f:
            map_metadata = yaml.safe_load(f)
        
        # Get image file path (relative to YAML file)
        image_file = map_metadata.get('image', '')
        if not os.path.isabs(image_file):
            yaml_dir = os.path.dirname(yaml_file)
            image_file = os.path.join(yaml_dir, image_file)
        
        # Update parameters from YAML
        self.resolution = map_metadata.get('resolution', self.resolution)
        origin = map_metadata.get('origin', [self.origin_x, self.origin_y, self.origin_theta])
        self.origin_x, self.origin_y, self.origin_theta = origin
        self.occupied_thresh = map_metadata.get('occupied_thresh', self.occupied_thresh)
        self.free_thresh = map_metadata.get('free_thresh', self.free_thresh)
        
        # Load image
        self.load_image_map(image_file)
        
        self.get_logger().info(f'Loaded map from {yaml_file}')

    def load_image_map(self, image_file):
        """Load map from image file.
        
        Args:
            image_file: Path to image file
        """
        # Load image
        img = Image.open(image_file).convert('L')  # Convert to grayscale
        self.map_width, self.map_height = img.size
        
        # Convert to numpy array
        img_array = np.array(img)
        
        # Convert to occupancy grid values (-1: unknown, 0: free, 100: occupied)
        # In image: 255 is free, 0 is occupied, 205 is unknown
        self.map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Normalize to 0-1 range
        normalized = img_array.astype(float) / 255.0
        
        # Convert to occupancy values
        self.map_data[normalized >= self.occupied_thresh] = 0  # Free
        self.map_data[normalized <= self.free_thresh] = 100  # Occupied
        mask = (normalized > self.free_thresh) & (normalized < self.occupied_thresh)
        self.map_data[mask] = -1  # Unknown
        
        self.get_logger().info(f'Loaded map from image: {self.map_width}x{self.map_height}')

    def create_empty_map(self, width, height):
        """Create an empty map.
        
        Args:
            width: Map width in cells
            height: Map height in cells
        """
        self.map_width = width
        self.map_height = height
        self.map_data = np.zeros((height, width), dtype=np.int8)
        self.get_logger().info(f'Created empty map: {width}x{height}')

    def publish_map(self):
        """Publish the map as an OccupancyGrid message."""
        if self.map_data is None:
            return
        
        # Create OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        # Map metadata
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = np.cos(self.origin_theta / 2)
        map_msg.info.origin.orientation.z = np.sin(self.origin_theta / 2)
        
        # Flatten map data (row-major order)
        map_msg.data = self.map_data.flatten().tolist()
        
        # Publish
        self.map_pub.publish(map_msg)
        
        # Publish map info
        info_msg = String()
        info_msg.data = f'Map: {self.map_width}x{self.map_height}, resolution: {self.resolution}m'
        self.map_info_pub.publish(info_msg)


def main(args=None):
    rclpy.init(args=args)
    map_parser = MapParser()
    
    try:
        rclpy.spin(map_parser)
    except KeyboardInterrupt:
        pass
    finally:
        map_parser.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
