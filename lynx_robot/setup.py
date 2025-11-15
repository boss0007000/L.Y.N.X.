from setuptools import setup
import os
from glob import glob

package_name = 'lynx_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LYNX Team',
    maintainer_email='user@example.com',
    description='ROS 2 package for L.Y.N.X. autonomous navigation robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = lynx_robot.motor_controller:main',
            'camera_node = lynx_robot.camera_node:main',
            'object_detector = lynx_robot.object_detector:main',
            'landmark_detector = lynx_robot.landmark_detector:main',
            'map_parser = lynx_robot.map_parser:main',
            'path_planner = lynx_robot.path_planner:main',
            'navigation_controller = lynx_robot.navigation_controller:main',
            'waypoint_selector = lynx_robot.waypoint_selector:main',
        ],
    },
)
