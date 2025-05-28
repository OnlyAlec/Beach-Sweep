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

# Intento de importar Hailo para aceleración
try:
    from hailo_platform.pyhailort import pyhailort
    HAILO_AVAILABLE = True
    HAILO_MODULE = pyhailort
    print("✅ ÉXITO: Hailo disponible usando hailo_platform.pyhailort")
except ImportError:
    try:
        from hailo_platform import pyhailort
        HAILO_AVAILABLE = True
        HAILO_MODULE = pyhailort
        print("✅ ÉXITO: Hailo disponible usando hailo_platform")
    except ImportError:
        HAILO_AVAILABLE = False
        HAILO_MODULE = None
        print("⚠️ ADVERTENCIA: Hailo no está disponible, usando CPU para inferencia")

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

        # 1. Cargar el modelo YOLO desde el paquete
        package_name = 'vision_ia_core'
        data_dir = os.path.join(get_package_share_directory(package_name), 'data')
        
        # Definir rutas de modelos por prioridad
        onnx_path = os.path.join(data_dir, 'yolo11n.onnx')
        pt_path = os.path.join(data_dir, 'yolo11n.pt')
        hef_path = os.path.join(data_dir, 'yolo11n.hef')

        # Verificar si hay aceleración por hardware disponible
        self.device = self._detect_best_device()
        
        # Cargar modelo según dispositivo disponible (prioridad: HEF > ONNX > PT)
        if self.device == 'hailo' and HAILO_AVAILABLE and os.path.exists(hef_path):
            # Usar modelo HEF para Hailo
            self.model = self._load_hailo_model(hef_path)
            self.get_logger().info(f'🚀 ACELERACIÓN: Modelo Hailo cargado desde: {hef_path}')
        elif os.path.exists(onnx_path):
            # Usar modelo ONNX (mejor rendimiento que PyTorch)
            self.model = YOLO(onnx_path)
            if self.device == 'cuda':
                self.model.to('cuda')
            self.get_logger().info(f'⚡ OPTIMIZADO: Modelo ONNX cargado desde: {onnx_path}')
        elif os.path.exists(pt_path):
            # Fallback a modelo PyTorch
            self.model = YOLO(pt_path)
            if self.device == 'cuda':
                self.model.to('cuda')
            self.get_logger().info(f'📦 ESTÁNDAR: Modelo PyTorch cargado desde: {pt_path}')
        else:
            raise FileNotFoundError(f"No se encontró ningún modelo en {data_dir}. "
                                  f"Verifica que exista yolo11n.onnx, yolo11n.pt o yolo11n.hef")
                
        self.get_logger().info(f'🔧 DISPOSITIVO: Usando {self.device} para inferencia')

    def _detect_best_device(self):
        """Detecta el mejor dispositivo disponible para inferencia"""
        # 1. Prioridad: Hailo AI Kit
        if HAILO_AVAILABLE:
            try:
                # Verificar si hay dispositivos Hailo disponibles usando pyhailort
                devices = HAILO_MODULE.Device.scan()
                if devices:
                    self.get_logger().info(f'🎯 DETECTADO: {len(devices)} dispositivo(s) Hailo disponible(s)')
                    return 'hailo'
            except Exception as e:
                self.get_logger().warn(f'⚠️ Error verificando Hailo: {str(e)}')
        
        # 2. Fallback: CUDA si está disponible
        try:
            import torch
            if torch.cuda.is_available():
                self.get_logger().info('🎯 DETECTADO: GPU CUDA disponible')
                return 'cuda'
        except ImportError:
            pass
        
        # 3. Último recurso: CPU
        self.get_logger().info('🔧 USANDO: CPU para inferencia (considera instalar Hailo para mayor velocidad)')
        return 'cpu'
    
    def _load_hailo_model(self, hef_path):
        """Carga un modelo en formato HEF para Hailo"""
        try:
            # Inicializar dispositivo Hailo
            devices = HAILO_MODULE.Device.scan()
            if not devices:
                raise RuntimeError("No se encontraron dispositivos Hailo")
            
            # Crear dispositivo Hailo (toma el primer dispositivo disponible)
            self.hailo_device = HAILO_MODULE.Device(device_info=devices[0])
            
            # Cargar el modelo HEF
            self.hailo_hef = HAILO_MODULE.HEF(hef_path)
            
            # Crear modelo de inferencia
            self.hailo_infer_model = self.hailo_device.create_infer_model(self.hailo_hef)
            
            self.get_logger().info(f'✅ Modelo Hailo cargado exitosamente desde: {hef_path}')
            return None  # No usamos el modelo YOLO estándar con Hailo
        except Exception as e:
            self.get_logger().error(f'❌ Error cargando modelo Hailo: {str(e)}')
            raise

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
