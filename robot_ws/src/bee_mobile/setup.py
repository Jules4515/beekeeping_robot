import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bee_mobile'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),        
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),        
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bee',
    maintainer_email='bee@todo.todo',
    description='Swerve kinematics and Joystick control for the bee_mobile robot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # General
            'pid_tuner = bee_mobile.pid_tuner:main',
            'odometry = bee_mobile.odometry:main',
            'wheel_state_converter = bee_mobile.wheel_state_converter:main',
            'mux_joystick = bee_mobile.mux_joystick:main',
            'unitree_imu_hotfix = bee_mobile.unitree_imu_hotfix:main',
            
            # Nav2
            'swerve_kinematics = bee_mobile.swerve_kinematics:main',
            'new_swerve_kinematics = bee_mobile.new_swerve_kinematics:main',
            'goal_manager = bee_mobile.goal_manager:main',

            # Docking
            'aruco_tag_detector = bee_mobile.aruco_tag_detector:main',
            'docking_controller = bee_mobile.docking_controller:main',
            'docking_tests = bee_mobile.docking_tests:main',
            'docking_tf_math_tester = bee_mobile.docking_tf_math_tester:main',

            # GPS
            'custom_gps_driver = bee_mobile.custom_gps_driver:main',
            'robot_footprint_publisher = bee_mobile.robot_footprint_publisher:main',
            'gps_health_monitor = bee_mobile.gps_health_monitor:main',
            'previsualization_gps = bee_mobile.previsualization_gps:main',
            'gps_monitor = bee_mobile.gps_monitor:main',
            'auto_gps_saver = bee_mobile.auto_gps_saver:main',
            'trajectory_recorder = bee_mobile.trajectory_recorder:main',
            'gps_mission = bee_mobile.gps_mission:main',            
        ],
    },
)
