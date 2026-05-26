import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('bee_mobile')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # SLAM creates its own map
    # map_file = os.path.join(
    #     get_package_share_directory('bee_mobile'),
    #     'maps',
    #     'tb3_sandbox_upscale.yaml'
    # )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            # 'map': map_file,
            'params_file': nav2_params,
            'use_sim_time': use_sim_time,
            'slam': 'True'           # <-- Deactivates AMCL and launch SLAM
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        nav2_cmd
    ])