#!/usr/bin/env python3
"""Path Planner Node for L.Y.N.X. Robot.

This node implements A* path planning algorithm for navigation.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import numpy as np
import heapq
import json


class PathPlanner(Node):
    """Plans paths using A* algorithm."""

    def __init__(self):
        super().__init__('path_planner')
        
        # Declare parameters
        self.declare_parameter('planning_method', 'astar')
        self.declare_parameter('goal_tolerance', 0.2)  # meters
        
        # Get parameters
        self.planning_method = self.get_parameter('planning_method').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # Map data
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        
        # Subscribe to map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        
        # Subscribe to goal requests (JSON: {"start": [x, y], "goal": [x, y]})
        self.goal_sub = self.create_subscription(
            String,
            'planning_request',
            self.planning_request_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.status_pub = self.create_publisher(String, 'planning_status', 10)
        
        self.get_logger().info(f'Path Planner initialized with {self.planning_method} algorithm')

    def map_callback(self, msg):
        """Store map data.
        
        Args:
            msg: OccupancyGrid message
        """
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        
        # Convert to 2D array
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        
        self.get_logger().debug('Map updated')

    def planning_request_callback(self, msg):
        """Handle path planning requests.
        
        Args:
            msg: String message with JSON data containing start and goal
        """
        try:
            request = json.loads(msg.data)
            start = request.get('start', [0, 0])
            goal = request.get('goal', [1, 1])
            
            self.get_logger().info(f'Planning path from {start} to {goal}')
            
            # Convert world coordinates to grid coordinates
            start_grid = self.world_to_grid(start[0], start[1])
            goal_grid = self.world_to_grid(goal[0], goal[1])
            
            # Plan path
            if self.planning_method == 'astar':
                path_grid = self.plan_astar(start_grid, goal_grid)
            else:
                path_grid = []
            
            if path_grid:
                # Convert back to world coordinates and publish
                self.publish_path(path_grid)
                status_msg = String()
                status_msg.data = f'Path found with {len(path_grid)} waypoints'
                self.status_pub.publish(status_msg)
                self.get_logger().info(status_msg.data)
            else:
                status_msg = String()
                status_msg.data = 'No path found'
                self.status_pub.publish(status_msg)
                self.get_logger().warn(status_msg.data)
                
        except Exception as e:
            self.get_logger().error(f'Error processing planning request: {str(e)}')

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates.
        
        Args:
            x: X coordinate in world frame (meters)
            y: Y coordinate in world frame (meters)
            
        Returns:
            Tuple of (grid_x, grid_y)
        """
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates.
        
        Args:
            grid_x: X coordinate in grid
            grid_y: Y coordinate in grid
            
        Returns:
            Tuple of (x, y) in world frame
        """
        x = grid_x * self.map_resolution + self.map_origin_x
        y = grid_y * self.map_resolution + self.map_origin_y
        return (x, y)

    def is_valid_cell(self, x, y):
        """Check if cell is valid and not occupied.
        
        Args:
            x: Grid x coordinate
            y: Grid y coordinate
            
        Returns:
            True if cell is valid and free
        """
        if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
            return False
        
        if self.map_data is None:
            return True
        
        # Cell is valid if it's free (0) or unknown (-1)
        return self.map_data[y, x] < 50

    def heuristic(self, a, b):
        """Calculate heuristic distance (Euclidean).
        
        Args:
            a: Tuple (x, y)
            b: Tuple (x, y)
            
        Returns:
            Euclidean distance
        """
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def plan_astar(self, start, goal):
        """Plan path using A* algorithm.
        
        Args:
            start: Start position (grid_x, grid_y)
            goal: Goal position (grid_x, grid_y)
            
        Returns:
            List of waypoints in grid coordinates
        """
        if self.map_data is None:
            self.get_logger().error('No map available for planning')
            return []
        
        if not self.is_valid_cell(start[0], start[1]):
            self.get_logger().error(f'Start position {start} is not valid')
            return []
        
        if not self.is_valid_cell(goal[0], goal[1]):
            self.get_logger().error(f'Goal position {goal} is not valid')
            return []
        
        # A* implementation
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        # 8-connected grid
        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            # Check if we reached the goal
            if self.heuristic(current, goal) < 1.0:  # Within 1 grid cell
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not self.is_valid_cell(neighbor[0], neighbor[1]):
                    continue
                
                # Cost is higher for diagonal moves
                move_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                tentative_g_score = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return []  # No path found

    def publish_path(self, path_grid):
        """Publish path as Path message.
        
        Args:
            path_grid: List of waypoints in grid coordinates
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for grid_x, grid_y in path_grid:
            # Convert to world coordinates
            x, y = self.grid_to_world(grid_x, grid_y)
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    
    try:
        rclpy.spin(path_planner)
    except KeyboardInterrupt:
        pass
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
