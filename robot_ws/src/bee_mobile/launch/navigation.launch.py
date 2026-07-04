# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# def generate_launch_description():
#     pkg_share = get_package_share_directory('bee_mobile')
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')

#     # Chemins fixes vers tes fichiers de configuration réels
#     nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
#     map_path = os.path.join(pkg_share, 'maps', 'carte_labo_2026-06-08_16.17.28.yaml')
#     sim_map_path = os.path.join(pkg_share, 'maps', 'tb3_sandbox_upscale.yaml')

#     # Lancement Nav2 en mode Autonomie Réelle
#     nav2_bringup = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
#         launch_arguments={
#             'map': map_path, # MONDE RÉEL
#             #'map': sim_map_path, # SIMULATION
#             'params_file': nav2_params,
#             'use_sim_time': 'False', # Crucial pour le matériel réel
#             'slam': 'False',         # On utilise la map existante pour localiser
#         }.items()
#     )

#     return LaunchDescription([
#         nav2_bringup
#     ])

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
    
    nav2_params = os.path.join(bee_mobile_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(bee_mobile_dir, 'maps', 'blank_small_map.yaml')
    ekf_config = os.path.join(bee_mobile_dir, 'config', 'ekf.yaml')
    
    # ============================================
    # PHASE 1 : Capteurs de base (démarrage immédiat)
    # ============================================
    
    # # GPS Driver
    # nmea_driver = Node(
    #     package='nmea_navsat_driver',
    #     executable='nmea_serial_driver',
    #     name='nmea_serial_driver',
    #     output='screen',
    #     parameters=[{
    #         'port': '/dev/ttyUSB_GPS',
    #         'baud': 115200,
    #         'publish_nmea_sentence': True,
    #     }]
    # )

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
    
    # # Heading Publisher
    # heading_publisher = Node(
    #     package='bee_mobile',
    #     executable='heading_publisher',
    #     name='heading_publisher',
    #     output='screen',
    # )

    # ============================================
    # VIRTUAL LIDAR - LiDAR fantôme pour le collision monitor
    # ============================================
    # virtual_lidar = Node(
    #     package='bee_mobile',
    #     executable='virtual_lidar',
    #     name='virtual_lidar',
    #     output='screen',
    #     parameters=[{'publish_rate': 1.0}]
    # )

    # ============================================
    # PHASE 2 : Localisation (après 2 secondes)
    # ============================================
    
    # EKF Local (délai de 2s pour laisser les capteurs démarrer)
    ekf_local = TimerAction(
        period=2.0,
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
        period=2.5,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_node_global',
                output='screen',
                parameters=[
                    ekf_config
                #     {
                #     'odom0_pose': [0.64, 0.0, 0.5, 0.0, 0.0, 0.0],}
                    ],
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

    # gps_corrector = Node(
    #     package='bee_mobile',
    #     executable='gps_corrector',
    #     name='gps_corrector',
    #     parameters=[{
    #         'offset_x': -0.64,  # Mesurez : distance centre → GPS vers l'AVANT
    #         'offset_y': 0.0,   # Mesurez : distance centre → GPS vers la GAUCHE
    #         'offset_z': 0.95,
    #     }]
    # )

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
        #heading_publisher,                               # heading_publisher.py
        #virtual_lidar,                                   # virtual_lidar.py
        
        # Phase 2 : Après délai
        ekf_local,
        ekf_global,
        robot_footprint,                                 # robot_footprint_publisher.py
        
        # Phase 3 : GPS Transform
        initialize_origin,                               # pour mapviz
        # gps_corrector,
        navsat_transform,
        
        # Phase 4 : Services et Navigation
        gps_health_monitor,                              # gps_health_monitor
        nav2_bringup,
    ])

# on utilise pour avoir des infos/action dans un terminal :
# gps_heading_display.py
# gps_health_monitor.py
# previsualisation_gps.py
# pid_tuner.py
