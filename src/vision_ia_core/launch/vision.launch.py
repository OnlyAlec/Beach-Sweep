#!/usr/bin/env python3
"""
Launch file para el paquete de visión
Inicia los nodos de cámara y detección YOLO
"""
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
            parameters=[{
                'video_source': '/home/alec/Documents/ULSA/ULSA_RoboBeach/src/vision_ia_core/data/video_latas.mp4',
                'frame_rate': 30.0,          # FPS del video
                'loop_video': True           # Repetir el video en bucle
            }],
            remappings=[
                ('robot/vision/frames', 'robot/vision/frames')
            ]
        ),
        
        # Nodo de detección YOLO
        Node(
            package='vision_ia_core',
            executable='detect_node',
            name='detect_node',
            output='screen',
            parameters=[{
                'model_path': '/home/alec/Documents/ULSA/ULSA_RoboBeach/src/vision_ia_core/data/yolo_model.pt',
                'confidence_threshold': 0.5,  # Umbral de confianza
                'detection_classes': ['can', 'bottle', 'container']  # Clases a detectar
            }],
            remappings=[
                ('robot/vision/frames', 'robot/vision/frames'),
                ('robot/vision/detections', 'robot/vision/detections')
            ]
        )
    ])
