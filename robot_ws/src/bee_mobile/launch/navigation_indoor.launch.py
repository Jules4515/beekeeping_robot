import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('bee_mobile')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Chemins fixes vers tes fichiers de configuration réels
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params_indoor.yaml')
    map_path = os.path.join(pkg_share, 'maps', 'carte_labo_2026-06-08_16.17.28.yaml')
    sim_map_path = os.path.join(pkg_share, 'maps', 'tb3_sandbox_upscale.yaml')

    # Lancement Nav2 en mode Autonomie Réelle
    nav2_bringup = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_path, # MONDE RÉEL
                    #'map': sim_map_path, # SIMULATION
                    'params_file': nav2_params,
                    'use_sim_time': 'False', # Crucial pour le matériel réel
                    'slam': 'True',         # On utilise la map existante pour localiser
                }.items()
            )
        ]
    )

    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/unilidar/cloud'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': 0.15,
            'max_height': 0.80,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.05,
            'range_min': 0.30,
            'range_max': 15.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }]
    )

    # Déclaration du noeud EKF Local
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[nav2_params],
    )

    return LaunchDescription([
        nav2_bringup,
        pointcloud_to_scan_node,
        ekf_node,
    ])