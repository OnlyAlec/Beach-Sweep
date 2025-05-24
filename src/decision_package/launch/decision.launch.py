#!/usr/bin/env python3
"""
Launch file para el paquete de decisiones
Inicia únicamente el nodo de lógica de decisiones
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='decision_package',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[{
                'safe_distance': 0.4,        # Distancia de seguridad en metros
                'max_capacity': 3,           # Capacidad máxima de latas
                'decision_rate': 5.0         # Frecuencia de toma de decisiones en Hz
            }],
            remappings=[
                ('robot/vision/detections', 'robot/vision/detections'),
                ('robot/sensors/distance', 'robot/sensors/distance'),
                ('robot/motor/commands', 'robot/motor/commands')
            ]
        )
    ])
