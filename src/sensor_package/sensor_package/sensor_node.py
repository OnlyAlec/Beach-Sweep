"""
Módulo Sensor para sistema de recolección de basura con ROS2

Responsabilidades:
1. Leer datos del sensor de distancia por ultrasonido (TF-Luna LiDAR)
2. Procesar y validar las mediciones de distancia
3. Publicar información de distancia en formato sensor_msgs/Range
4. Mantener comunicación serie con el hardware del sensor
5. Proporcionar datos críticos para navegación y detección de obstáculos

Tópicos:
- Publica en: /robot/sensors/distance (sensor_msgs/Range)
"""

import serial
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        # QoS Profile for sensor data (distance sensor)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Sensor data often uses BEST_EFFORT
            durability=DurabilityPolicy.VOLATILE,      # No need for durability for volatile sensor readings
            history=HistoryPolicy.KEEP_LAST,         # Keep only the latest readings
            depth=10                                   # A common depth for sensor data
        )

        # -- 1. Crear Publicador Topic "robot/sensors/distance" que manda un Range y guarda los últimos 10  
        self.publisher_ = self.create_publisher(Range, 'robot/sensors/distance', qos_sensor_data)
        
        # -- 2. Abrir puerto e inicializar variables 
        self.serial_port = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
        time.sleep(2)  # espera para que el sensor se estabilice
        self.distancia_metros = 0.0
        
        # -- 3. Crear temporizador para publicar distancia
        timer_period = 1.0 / 10 # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
                
    # -- Se activa cuando se llama al temporizador
    def timer_callback(self):
        # -- 4. Leer Distancia cuando se llamado por el temporizador 
        if self.serial_port.in_waiting >= 9:
            data = self.serial_port.read(9)
            if data[0] == 0x59 and data[1] == 0x59:
                checksum = sum(data[0:8]) & 0xFF
                if checksum == data[8]:
                    distancia = data[2] + data[3] * 256  # en milímetros
                    self.distancia_metros = distancia / 1000.0
        
                    # 5. Publicar distancia como un mensaje Range
                    msg = Range()
                    msg.radiation_type = Range.INFRARED
                    msg.field_of_view = 0.1  # radianes
                    msg.min_range = 0.02     # metros
                    msg.max_range = 12.0     # metros
                    msg.range = self.distancia_metros
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "distance_sensor"
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'>> Publicando distancia: {msg.range:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Nodo interrumpido por el usuario.')
    finally: 
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
