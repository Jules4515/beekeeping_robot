import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bee_mobile_dir = get_package_share_directory('bee_mobile')
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
        default_value=os.path.join(bee_mobile_dir, 'config', 'nav2_params_outdoor_swerve.yaml'),
        description='Full path to the ROS 2 parameters file to use for all Nav2 nodes'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Fixed configuration paths 
    map_file = os.path.join(bee_mobile_dir, 'maps', 'blank_small_map.yaml')
    ekf_config = os.path.join(bee_mobile_dir, 'config', 'ekf.yaml')
    
    # ========================================================================
    # PHASE 1: Base Sensors (Immediate start) 
    # ========================================================================

    custom_gps_driver = Node(
        package='bee_mobile',
        executable='custom_gps_driver',
        name='custom_gps_driver',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB_GPS',
            'baud': 115200,
        }]
    )

    # ========================================================================
    # PHASE 2: Localization (Starts after 3 seconds to let sensors boot) 
    # ========================================================================
    
    # Local EKF 
    ekf_local = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_node',
                output='screen',
                parameters=[ekf_config],
            )
        ]
    )
    
    # Global EKF 
    ekf_global = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_node_global',
                output='screen',
                remappings=[
                    ('/odometry/filtered', '/odometry/global'),
                ],
                parameters=[ekf_config],
            )
        ]
    )
    
    # ========================================================================
    # PHASE 3: GPS Transformation (Starts after 4/5 seconds) 
    # ========================================================================

    # Navsat transform (waits for gps_tf to be ready) 
    navsat_transform = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform_node',
                output='screen',
                parameters=[ekf_config],
                remappings=[
                    ('/gps/fix', '/fix'),
                    ('/imu', '/heading_imu'),
                    ('/odometry/filtered', '/odometry/global'),
                ]
            )
        ]
    )
            
    # ========================================================================
    # PHASE 4: Services and Navigation (Starts after 7/10 seconds) 
    # ========================================================================
    
    gps_health_monitor = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='bee_mobile',
                executable='gps_health_monitor',
                name='gps_health_monitor',
                output='screen',
                parameters=[{
                    'min_gps_status': 2,
                    'gps_timeout': 2.0,
                }]
            )
        ]
    )
    
    # Nav2 Bringup (Waits until system is stable) 
    nav2_bringup = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'params_file': params_file,
                    'use_sim_time': use_sim_time,
                    'slam': 'False',
                    'amcl': 'False',
                    'run_amcl': 'False',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        
        # Phase 1: Immediate 
        custom_gps_driver,
        
        # Phase 2: Delayed 
        ekf_local,
        ekf_global,
        
        # Phase 3: GPS Transform 
        navsat_transform,
        
        # Phase 4: Services & Navigation 
        gps_health_monitor,
        nav2_bringup,
    ])