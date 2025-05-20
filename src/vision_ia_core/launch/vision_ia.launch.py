# ==================================================================================
# VISION IA LAUNCH FILE
# ==================================================================================
# Se encarga de iniciar los nodos de cámara y detección.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_ia_core',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),
        Node(
            package='vision_ia_core',
            executable='detect_node',
            name='detect_node',
            output='screen',
        ),
    ])
