# =================================================
# SENSOR NODE
# =================================================

import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        # -- 1. Crear Publicador Topic "sensor_distance" que manda un flotante y guarda los últimos 10  
        self.publisher_ = self.create_publisher(Float32, 'sensor_distance', 10)
        
        # -- 2. Abrir puerto e inicializar variables 
        self.serial_port = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
        time.sleep(2)  # espera para que el sensor se estabilice
        self.distancia_metros = 0.0
        
        # -- 3. Crear temporizador para publicar distancia
        timer_period = 1.0 / 10 # 10 Hz
        self.timer = self.create_time(timer_period, self.timer_callback)
                
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
        
                    # 5. Publicar distancia como un mensaje con un flotante
                    msg = Float32()
                    msg.data = self.distancia_metros
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'>> Publicando distancia: {msg.data:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo interrumpido por teclado")
    finally: 
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
