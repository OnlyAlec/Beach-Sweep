#!/usr/bin/env python3
"""
Launch file para el paquete de sensores
Inicia únicamente el nodo del sensor TF-Luna LiDAR
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_package',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
            parameters=[{
                'distance_threshold': 0.4,  # Distancia mínima de seguridad en metros
                'publish_rate': 10.0        # Frecuencia de publicación en Hz
            }],
            remappings=[
                ('robot/sensors/distance', 'robot/sensors/distance')
            ]
        )
    ])
