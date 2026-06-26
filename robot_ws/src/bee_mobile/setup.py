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
            'pid_tuner = bee_mobile.pid_tuner:main',
            'wheel_state_converter = bee_mobile.wheel_state_converter:main',
            'swerve_kinematics = bee_mobile.swerve_kinematics:main',
            'sim_bridge = bee_mobile.sim_bridge:main',
            'mux_joystick_node = bee_mobile.mux_joystick_node:main',
            'waypoint_commander = bee_mobile.waypoint_commander:main',
            'odometry = bee_mobile.odometry:main',
            'docking_controller = bee_mobile.docking_controller:main',
            'docking_debug_cli = bee_mobile.docking_debug_cli:main',
            'heading_publisher = bee_mobile.heading_publisher:main',
            'robot_footprint_publisher = bee_mobile.robot_footprint_publisher:main',
            'gps_heading_display = bee_mobile.gps_heading_display:main',
            'custom_gps_driver = bee_mobile.custom_gps_driver:main',
            'distance_to_aruco = bee_mobile.distance_to_aruco:main',
            'docking_tf_math_tester = bee_mobile.docking_tf_math_tester:main',
            'compressed_aruco_node = bee_mobile.compressed_aruco_node:main',
            'docking_tests = bee_mobile.docking_tests:main'
        ],
    },
)
