# """
# Launch file for bee_mobile Navigation and Mapping.

# Usage combinations:
# 1. simulation:=false mapping_mode:=true  | Real-world exploration: Runs async SLAM only with hardware clock to build a map.
# 2. simulation:=false mapping_mode:=false | Real-world autonomy: Runs Nav2 + AMCL only with a pre-saved map and hardware clock.
# 3. simulation:=true mapping_mode:=true   | Simulated exploration: Runs async SLAM only using a virtual clock (e.g., Gazebo).
# 4. simulation:=true mapping_mode:=false  | Simulated autonomy: Runs Nav2 + AMCL only using a virtual clock for safe parameter testing.
# """
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PythonExpression
# from launch.conditions import IfCondition, UnlessCondition
# from launch_ros.actions import Node

# def generate_launch_description():
#     pkg_share = get_package_share_directory('bee_mobile')
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')

#     # --- 1. Arguments Definition ---
#     sim_arg = DeclareLaunchArgument(
#         'simulation',
#         default_value='false',
#         description='Set to true to use simulation clock (Gazebo/Loopback)'
#     )
#     is_sim = LaunchConfiguration('simulation')

#     mapping_mode_arg = DeclareLaunchArgument(
#         'mapping_mode',
#         default_value='false',
#         description='Set to true to run SLAM only. Set to false to run Nav2 with a saved map.'
#     )
#     mapping_mode = LaunchConfiguration('mapping_mode')

#     # --- 2. File Paths (Dynamic Map Selection) ---
#     nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
#     real_map_path = os.path.join(pkg_share, 'maps', 'carte_labo_2026-06-08_16.17.28.yaml')
#     sim_map_path = os.path.join(pkg_share, 'maps', 'tb3_sandbox_upscale.yaml')

#     # --- 3. Nodes Configuration ---

#     # Nav2 bringup when using the simulation map (mapping_mode:=false, simulation:=true)
#     nav2_cmd_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
#         condition=IfCondition(PythonExpression(["'", mapping_mode, "'.lower() == 'false' and '", is_sim, "'.lower() == 'true'"])),
#         launch_arguments={
#             'map': sim_map_path,
#             'params_file': nav2_params,
#             'use_sim_time': is_sim,
#             'slam': 'False',
#         }.items()
#     )

#     # Nav2 bringup when using the real map (mapping_mode:=false, simulation:=false)
#     # WHY: Setting slam to 'False' automatically disables slam_toolbox and forces Nav2 
#     # to spin up amcl and map_server to localize on the static real_map_path.
#     nav2_cmd_real = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
#         condition=IfCondition(PythonExpression(["'", mapping_mode, "'.lower() == 'false' and '", is_sim, "'.lower() == 'false'"])),
#         launch_arguments={
#             'map': real_map_path,
#             'params_file': nav2_params,
#             'use_sim_time': is_sim,
#             'slam': 'False'
#         }.items()
#     )

#     # --- 4. Custom SLAM Toolbox Stack (Runs ONLY when mapping_mode is true) ---
#     slam_node = Node(
#         condition=IfCondition(mapping_mode),
#         package='slam_toolbox',
#         executable='async_slam_toolbox_node',
#         name='slam_toolbox',
#         namespace='',
#         output='screen',
#         # Load YAML to preserve Swerve drift limits and QoS configurations
#         parameters=[nav2_params],
#         #arguments=['--ros-args', '--log-level', 'debug']
#     )

#     configure_slam = TimerAction(
#         condition=IfCondition(mapping_mode),
#         period=3.0,
#         actions=[ExecuteProcess(
#             cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
#             output='screen'
#         )]
#     )

#     activate_slam = TimerAction(
#         condition=IfCondition(mapping_mode),
#         period=10.0,
#         actions=[ExecuteProcess(
#             cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
#             output='screen'
#         )]
#     )

#     return LaunchDescription([
#         sim_arg,
#         mapping_mode_arg,
#         nav2_cmd_sim,
#         nav2_cmd_real,
#         slam_node,
#         configure_slam,
#         activate_slam
#     ])

# Simulation

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# def generate_launch_description():
#     pkg_share = get_package_share_directory('bee_mobile')
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')

#     # --- 2. File Paths (Dynamic Map Selection) ---
#     nav2_params = os.path.join(pkg_share, 'config', 'simple_nav2_params.yaml')
#     sim_map_path = os.path.join(pkg_share, 'maps', 'tb3_sandbox_upscale.yaml')

#     # --- 3. Nodes Configuration ---

#     # Nav2 bringup when using the simulation map (mapping_mode:=false, simulation:=true)
#     nav2_cmd_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
#         launch_arguments={
#             'map': sim_map_path,
#             'params_file': nav2_params,
#             'use_sim_time': 'False',
#             'slam': 'False',
#         }.items()
#     )

#     return LaunchDescription([
#         nav2_cmd_sim
#     ])

# Pas simulation
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('bee_mobile')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Chemins fixes vers tes fichiers de configuration réels
    nav2_params = os.path.join(pkg_share, 'config', 'simple_nav2_params.yaml')
    map_path = os.path.join(pkg_share, 'maps', 'carte_labo_2026-06-08_16.17.28.yaml')

    # Lancement Nav2 en mode Autonomie Réelle
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_path,
            'params_file': nav2_params,
            'use_sim_time': 'False', # Crucial pour le matériel réel
            'slam': 'False',         # On utilise la map existante pour localiser
        }.items()
    )

    return LaunchDescription([
        nav2_bringup
    ])