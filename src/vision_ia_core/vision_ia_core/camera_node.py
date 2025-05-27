#!/usr/bin/env python3
"""
Nodo de Cámara para el sistema RoboBeach

Responsabilidades:
1. Capturar frames de cámara usando Picamera2 (Raspberry Pi Camera)
2. Convertir frames a mensajes ROS2 Image
3. Publicar stream de imágenes para procesamiento de detección
4. Mantener consistencia en la frecuencia de frames
5. Gestionar limpieza de recursos de cámara

Tópicos publicados:
- robot/vision/frames (sensor_msgs/Image): Stream de frames de cámara
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # QoS Profile for sensor data (camera frames)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Match common subscriber depth
        )

        self.publisher_ = self.create_publisher(Image, 'robot/vision/frames', qos_sensor_data)
        self.bridge = CvBridge()

        # 1. Inicializar Picamera2
        try:
            self.picam2 = Picamera2()
            
            # 2. Crear configuración de preview
            preview_config = self.picam2.create_preview_configuration(
                main={"size": (1280, 720)},  # resolución principal
                lores={"size": (320, 240)}   # resolución de bajo coste opcional
            )
            self.picam2.configure(preview_config)
            
            # 3. Arrancar la cámara
            self.picam2.start()
            self.get_logger().info('✅ EXITO: Cámara Picamera2 inicializada correctamente')
            
        except Exception as e:
            self.get_logger().error(f'❌ ERROR: No se pudo inicializar Picamera2: {str(e)}')
            exit(1)

        # 3. Crear un temporizador para publicar frames a 10 fps
        timer_period = 1/60
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # -- Se activa cada vez que se llama al temporizador --
    def timer_callback(self):
        try:
            # Capturar frame como array NumPy en formato RGB
            frame = self.picam2.capture_array()
            
            # Convertir de RGB a BGR para mantener compatibilidad con OpenCV
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # 4. Publicar el frame como un mensaje de tipo Image
            msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
            self.publisher_.publish(msg)
            # self.get_logger().info('>> Frame publicado')-
            
        except Exception as e:
            self.get_logger().error(f'❌ ERROR capturando frame: {str(e)}')
            self.get_logger().info('Cerrando nodo...')
            self.cleanup()
            rclpy.shutdown()
    
    def cleanup(self):
        """Limpia recursos de la cámara"""
        try:
            if hasattr(self, 'picam2'):
                self.picam2.stop()
                self.get_logger().info('Cámara Picamera2 detenida correctamente')
        except Exception as e:
            self.get_logger().error(f'Error al limpiar recursos: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 ERROR: Nodo interrumpido por teclado.')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
