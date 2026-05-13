import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    # Lance le node slam_toolbox en mode mapping
    # Il démarre en état "unconfigured" et attend d'être activé
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_frame': 'odom',           # frame odométrie dans le TF tree
            'map_frame': 'map',             # frame de la carte publiée
            'base_frame': 'base_footprint', # racine du robot dans le TF tree
            'scan_topic': '/scan',          # topic lidar
            'resolution': 0.15,             # À 12m (distance max de détection du lidar C1) : d = 2π x 12 x (0.72/360) = 0.1508m = 15.08cm, 0.72 : résolution angulaire du lidar C1
        }]
    )

    # Après 3 secondes, configure le node (charge les paramètres)
    # Équivalent de : ros2 lifecycle set /slam_toolbox configure
    configure = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
            output='screen'
        )]
    )

    # Après 5 secondes, active le node (commence à publier sur /map)
    # Équivalent de : ros2 lifecycle set /slam_toolbox activate
    activate = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
            output='screen'
        )]
    )

    return LaunchDescription([
        slam_node,
        configure,  # déclenché à t+3s
        activate,   # déclenché à t+5s
    ])