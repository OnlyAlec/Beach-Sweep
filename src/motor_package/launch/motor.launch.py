#!/usr/bin/env python3
"""
Launch file para el paquete de motores
Inicia únicamente el nodo de control de motores
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_package',
            executable='motor_node',
            name='motor_controller',
            output='screen',
            parameters=[{
                'wheel_speed_default': 50,   # Velocidad por defecto de las ruedas (0-100)
                'broom_speed_default': 75,   # Velocidad por defecto de la escoba (0-100)
                'servo_update_rate': 0.1     # Frecuencia de actualización de servos en segundos
            }],
            remappings=[
                ('robot/motor/commands', 'robot/motor/commands')
            ]
        )
    ])
