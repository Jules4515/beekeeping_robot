import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Define paths to YOUR specific configuration files
    nav2_params = os.path.join(
        get_package_share_directory('bee_mobile'),
        'config',
        'nav2_params.yaml'
    )
    
    # Path to the map file (uncomment when you have saved your map using map_saver_cli)
    map_file = os.path.join(
        get_package_share_directory('bee_mobile'),
        'maps',
        'my_map.yaml'
    )

    # 2. Find the official nav2_bringup package directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 3. Return the launch description
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            # Pass the arguments to the Nav2 launch file
            launch_arguments={
                'map': map_file,  # Uncomment this line when your map file is ready
                'params_file': nav2_params,
                'use_sim_time': 'False'
            }.items()
        )
    ])