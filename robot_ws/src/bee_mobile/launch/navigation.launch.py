import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('bee_mobile'),
        'config',
        'nav2_params.yaml'
    )
    map_file = os.path.join(
        get_package_share_directory('bee_mobile'),
        'maps',
        'my_map.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='nav2_bringup',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': False}],
            arguments=['map:=', map_file]
        )
    ])