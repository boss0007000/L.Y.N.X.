"""Launch file for L.Y.N.X. Robot system."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for full robot system."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('lynx_robot')
    
    # Declare launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_dir, 'maps', 'example_map.yaml'),
        description='Path to map file'
    )
    
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to start camera node'
    )
    
    # Launch configuration
    map_file = LaunchConfiguration('map_file')
    use_camera = LaunchConfiguration('use_camera')
    
    # Nodes
    motor_controller = Node(
        package='lynx_robot',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'motor_config.yaml')
        ]
    )
    
    camera_node = Node(
        package='lynx_robot',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'camera_config.yaml')
        ],
        condition=IfCondition(use_camera)
    )
    
    object_detector = Node(
        package='lynx_robot',
        executable='object_detector',
        name='object_detector',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'detection_config.yaml')
        ]
    )
    
    landmark_detector = Node(
        package='lynx_robot',
        executable='landmark_detector',
        name='landmark_detector',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'landmark_config.yaml')
        ]
    )
    
    map_parser = Node(
        package='lynx_robot',
        executable='map_parser',
        name='map_parser',
        output='screen',
        parameters=[
            {'map_file': map_file},
            os.path.join(pkg_dir, 'config', 'map_config.yaml')
        ]
    )
    
    path_planner = Node(
        package='lynx_robot',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'planner_config.yaml')
        ]
    )
    
    navigation_controller = Node(
        package='lynx_robot',
        executable='navigation_controller',
        name='navigation_controller',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'navigation_config.yaml')
        ]
    )
    
    waypoint_selector = Node(
        package='lynx_robot',
        executable='waypoint_selector',
        name='waypoint_selector',
        output='screen',
        parameters=[
            {'use_interactive_mode': True}
        ]
    )
    
    return LaunchDescription([
        map_file_arg,
        use_camera_arg,
        motor_controller,
        camera_node,
        object_detector,
        landmark_detector,
        map_parser,
        path_planner,
        navigation_controller,
        waypoint_selector
    ])
