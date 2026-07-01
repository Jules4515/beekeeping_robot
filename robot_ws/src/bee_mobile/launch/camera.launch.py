from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Chemin vers le fichier de calibration validé
    camera_info_yaml = os.path.join(
        get_package_share_directory('bee_mobile'), 
        'config', 
        'camera_info.yaml'
    )

    return LaunchDescription([
        
        # 1. Pilote de la caméra (Fréquence nominale à 30 FPS en local)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
                'image_width': 640,
                'image_height': 480,
                'camera_frame_id': 'camera_link',
                'camera_info_url': 'file://' + camera_info_yaml,
                'exposure_auto': 1,       # 1 correspond souvent à un mode manuel ou priorité vitesse selon le pilote
                'exposure_absolute': 20,
                'gain': 10,
                'qos_reliability': 'best_effort',
                'qos_history': 'keep_last',
                'qos_depth': 1,
            }]
        ),
    ])