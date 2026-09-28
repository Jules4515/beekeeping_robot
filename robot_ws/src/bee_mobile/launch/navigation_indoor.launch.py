import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('bee_mobile')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ========================================================================
    # 1. LAUNCH ARGUMENTS
    # ========================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params_indoor_swerve.yaml'),
        description='Full path to the ROS 2 parameters file to use for all Nav2 nodes'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Fixed path to the existing map 
    map_path = os.path.join(pkg_share, 'maps', 'carte_labo_2026-06-08_16.17.28.yaml')

    # ========================================================================
    # 2. NODES & ACTIONS
    # ========================================================================

    # EKF Local Node for odometry fusion 
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[params_file],
    )

    # Nav2 Bringup (Real-world autonomy mode) 
    nav2_bringup = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_path,        # REAL WORLD map
                    'params_file': params_file,
                    'use_sim_time': use_sim_time, # Crucial for real hardware 
                    'slam': 'True',         # Use existing map for localization 
                }.items()
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        ekf_node,
        nav2_bringup,
    ])