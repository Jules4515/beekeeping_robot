import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    pkg_share = get_package_share_directory('bee_mobile')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # --- Arguments ---
    sim_arg = DeclareLaunchArgument(
        'simulation',
        default_value='False',
        description='Set to True to use simulation time and disable SLAM'
    )
    is_sim = LaunchConfiguration('simulation')

    # Enable SLAM only in real world. In sim, map->odom is handled by static TF.
    use_slam = PythonExpression(["'False' if '", is_sim, "' == 'True' else 'True'"])

    # --- Map & Params ---
    map_file = os.path.join(pkg_share, 'maps', 'tb3_sandbox_upscale.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # --- Nav2 Bringup ---
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params,
            'use_sim_time': 'False',
            'slam': use_slam
        }.items()
    )

    return LaunchDescription([
        sim_arg,
        nav2_cmd
    ])