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

    # Lancement Nav2 en mode Autonomie Réelle
    nav2_bringup = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_path, # MONDE RÉEL
                    'params_file': nav2_params,
                    'use_sim_time': 'False', # Crucial pour le matériel réel
                    'slam': 'True',         # On utilise la map existante pour localiser
                }.items()
            )
        ]
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
        ekf_node,
    ])