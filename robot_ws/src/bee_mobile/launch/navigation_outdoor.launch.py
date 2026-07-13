import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bee_mobile_dir = get_package_share_directory('bee_mobile')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    nav2_params = os.path.join(bee_mobile_dir, 'config', 'nav2_params_outdoor.yaml')
    map_file = os.path.join(bee_mobile_dir, 'maps', 'blank_small_map.yaml')
    ekf_config = os.path.join(bee_mobile_dir, 'config', 'ekf.yaml')
    
    # ============================================
    # PHASE 1 : Capteurs de base (démarrage immédiat)
    # ============================================

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

    # ============================================
    # PHASE 2 : Localisation (après 2 secondes)
    # ============================================
    
    # EKF Local (délai de 2s pour laisser les capteurs démarrer)
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
    
    # EKF Global (délai de 2.5s)
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
                ]
            )
        ]
    )

    robot_footprint = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='bee_mobile',
                executable='robot_footprint_publisher',
                name='robot_footprint_publisher',
                output='screen',
            )
        ]
    )
    
    # ============================================
    # PHASE 3 : Transformation GPS (après 5 secondes)
    # ============================================
    
    # Pour l'initialisation de mapviz
    initialize_origin = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='swri_transform_util',
                executable='initialize_origin.py',
                name='initialize_origin',
                output='screen',
                remappings=[('fix', '/fix')],
                parameters=[{
                    'local_xy_frame': 'map',
                    'local_xy_origin': 'auto',
                }]
            )
        ]
    )

    # Navsat transform qui attend que gps_tf soit prêt
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
            
        
    # ============================================
    # PHASE 4 : Services et Navigation (après 8 secondes)
    # ============================================
    
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
    
    # Nav2 Bringup (après 10 secondes, quand tout est stable)
    nav2_bringup = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'params_file': nav2_params,
                    'use_sim_time': 'False',
                    'slam': 'False',
                    'amcl': 'False',
                    'run_amcl': 'False',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        # Phase 1 : Immédiat
        custom_gps_driver,
        
        # Phase 2 : Après délai
        ekf_local,
        ekf_global,
        robot_footprint,                                 # robot_footprint_publisher.py
        
        # Phase 3 : GPS Transform
        initialize_origin,                               # pour mapviz
        navsat_transform,
        
        # Phase 4 : Services et Navigation
        gps_health_monitor,
        nav2_bringup,
    ])