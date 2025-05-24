"""
Nodo de Cámara para el sistema RoboBeach

Responsabilidades:
1. Capturar frames de video desde archivo o cámara en vivo
2. Convertir frames a mensajes ROS2 Image
3. Publicar stream de imágenes para procesamiento de detección
4. Mantener consistencia en la frecuencia de frames
5. Gestionar bucle automático del video para testing continuo

Tópicos publicados:
- robot/vision/frames (sensor_msgs/Image): Stream de frames de video
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.publisher_ = self.create_publisher(Image, 'robot/vision/frames', 10)
        self.bridge = CvBridge()

        # 1. Cargar el video desde el paquete
        package_name = 'vision_ia_core'
        data_dir = os.path.join(get_package_share_directory(package_name), 'data')
        video_path = os.path.join(data_dir, 'video.mp4')

        # 2. Verificar si el video existe y abrirlo
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f'❌ ERROR: No se pudo abrir el video: {video_path}')
            exit(1)

        self.get_logger().info(f'✅ EXITO: Reproduciendo video desde... {video_path}')

        # 3. Crear un temporizador para publicar frames a 10 fps
        timer_period = 1/10  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # -- Se activa cada vez que se llama al temporizador --
    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().info('Video terminado, cerrando nodo...')
            self.cap.release()
            rclpy.shutdown()
            return
        
        # 4. Publicar el frame como un mensaje de tipo Image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('>> Frame publicado')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 ERROR: Nodo interrumpido por teclado.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
