#!/usr/bin/env python3
"""
Nodo de Detección YOLO para el sistema RoboBeach

Responsabilidades:
1. Recibir frames de video del nodo de cámara
2. Aplicar modelo YOLO entrenado para detectar objetos de interés
3. Filtrar detecciones por clase (latas, botellas, contenedores)
4. Calcular coordenadas y área de objetos detectados
5. Publicar detecciones estructuradas para toma de decisiones

Clases de objetos detectados:
- can: Latas de bebidas (objetivo de recolección)
- bottle: Botellas plásticas (objetivo de recolección)  
- container: Contenedores rojos (objetivo de descarga)

Tópicos suscritos:
- robot/vision/frames (sensor_msgs/Image): Frames de la cámara

Tópicos publicados:
- robot/vision/detections (vision_ia_msgs/Detections): Objetos detectados con coordenadas
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from robot_interfaces.msg import Detection, Detections
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class DetectNode(Node):
    def __init__(self):
        super().__init__('detect_node')
        
        # QoS Profile for subscribing to sensor data (camera frames)
        qos_sensor_data_subscriber = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, # Match publisher (CameraNode)
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS Profile for publishing detection results
        # Detections might be considered more critical than raw sensor data,
        # but BEST_EFFORT is often fine if the decision node can handle occasional missed messages.
        # If every detection is critical, consider RELIABLE.
        qos_detection_publisher = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Or RELIABLE if needed
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10 
        )

        self.subscription = self.create_subscription(
            Image,
            'robot/vision/frames',
            self.listener_callback,
            qos_sensor_data_subscriber)
        self.subscription # prevent unused variable warning

        self.publisher_ = self.create_publisher(Detections, 'robot/vision/detections', qos_detection_publisher)
        
        self.bridge = CvBridge()

        # 1. Cargar el modelo YOLOv8 desde el paquete
        package_name = 'vision_ia_core'
        data_dir = os.path.join(get_package_share_directory(package_name), 'data')
        model_path = os.path.join(data_dir, 'yolo11n.pt')

        self.model = YOLO(model_path)
        self.get_logger().info(f'✅ EXITO: Modelo YOLO cargado desde: {model_path}')

    # -- Se activa cuando se recibe un mensaje tipo Image --
    def listener_callback(self, msg):
        # -- 2. Convertir mensaje ROS a imagen OpenCV --
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'❌ ERROR: Error al convertir imagen... {e}')
            return

        height, width, _ = frame.shape

        # -- 3. Hacer predicción con YOLOv8 sobre el frame --
        results = self.model(frame)[0]
        detections = []

        # -- 4. Procesar cada detección extrayendo su información --
        for box, conf, cls_id in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
            conf = float(conf)

            # Ignorar detecciones con baja confianza
            if conf < 0.5:
                continue  

            x1, y1, x2, y2 = box.cpu().numpy()
            w = x2 - x1
            h = y2 - y1

            # Normalizar coordenadas
            x_norm = x1 / width
            y_norm = y1 / height
            w_norm = w / width
            h_norm = h / height

            cls_id = int(cls_id.cpu().numpy())
            class_name = self.model.names[cls_id]

            # Crear objeto Detection y añadirlo a la lista
            detection = Detection()
            detection.class_name = class_name
            detection.x = x_norm
            detection.y = y_norm
            detection.width = w_norm
            detection.height = h_norm
            detection.confidence = conf
            detection.class_id = cls_id

            detections.append(detection)

            # Dibujar bounding box y texto en la imagen para visualización
            start_point = (int(x1), int(y1))
            end_point = (int(x2), int(y2))
            color = (0, 255, 0)
            cv2.rectangle(frame, start_point, end_point, color, 2)
            cv2.putText(frame, f'{class_name} {conf:.2f}', (int(x1), int(y1)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # -- 5. Comprobar si hay detecciones --
        if len(detections) == 0:
            self.get_logger().warn('⚠️ PRECAUCION: No se detectó ningún objeto en este frame.')

        # -- 6. Publicar las detecciones --
        msg_out = Detections()
        msg_out.header = msg.header  
        msg_out.detections = detections
        self.publisher_.publish(msg_out)
        self.get_logger().info(f'* Publicadas {len(detections)} detecciones.')

        # -- 7. Mostrar imagen con detecciones si hay GUI Local --
        cv2.imshow("Detections", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 ERROR: Nodo interrumpido por teclado.')
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
