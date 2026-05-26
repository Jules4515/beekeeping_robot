import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 460800,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_frequency': 10.0,  # ADDED: reduce lidar frequency to 5 Hz to avoid message filter overflow
            }]
        ),
    ])