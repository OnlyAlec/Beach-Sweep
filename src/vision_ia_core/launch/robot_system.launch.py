# ==================================================================================
# ROBOT SYSTEM LAUNCH FILE
# ==================================================================================
# Lanza todos los nodos del sistema de robot de limpieza de playa

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de cámara
        Node(
            package='vision_ia_core',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        # Nodo de detección
        Node(
            package='vision_ia_core',
            executable='detect_node',
            name='detect_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        # Nodo de sensores
        Node(
            package='sensor_package',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        # Nodo de decisiones
        Node(
            package='decision_package',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        # Nodo de motores
        Node(
            package='motor_package',
            executable='motor_node',
            name='motor_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
    ])
