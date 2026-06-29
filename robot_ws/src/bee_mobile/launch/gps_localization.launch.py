from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Chemin vers ton fichier yaml unique
    config_dir = os.path.join(get_package_share_directory('ton_package_name'), 'config')
    ekf_params_file = os.path.join(config_dir, 'dual_ekf_navsat.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            parameters=[ekf_params_file],
            remappings=[('odometry/filtered', 'odometry/local')]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            parameters=[ekf_params_file],
            remappings=[('odometry/filtered', 'odometry/global')]
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=[ekf_params_file],
            remappings=[('gps/fix', '/gps/fix'), ('imu', '/imu/data')]
        )
    ])