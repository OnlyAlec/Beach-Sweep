"""
Módulo Sensor para sistema de recolección de basura con ROS2

Responsabilidades:
1. Leer datos del sensor de distancia por ultrasonido (TF-Luna LiDAR)
2. Procesar y validar las mediciones de distancia
3. Publicar información de distancia en formato sensor_msgs/Range
4. Mantener comunicación serie con el hardware del sensor (UART)
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

SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        self.serial_handle = None
        self.distancia_metros = 0.0

        # QoS Profile for sensor data
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(Range, 'robot/sensors/distance', qos_sensor_data)
        
        try:
            self.serial_handle = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"Successfully opened serial port {SERIAL_PORT} at {BAUD_RATE} baud.")
            time.sleep(0.1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {SERIAL_PORT}: {e}")
            rclpy.shutdown()
            raise SystemExit(f"Fatal Serial error: {e}")

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
                
    def timer_callback(self):
        try:
            # TF-Luna UART data frame (9 bytes):
            # Header (0x59, 0x59), Dist_L, Dist_H, Strength_L, Strength_H, Temp_L, Temp_H, Checksum
            if self.serial_handle.in_waiting >= 9:
                header = self.serial_handle.read(2)
                if header == b'\x59\x59':  # Check for TF-Luna header
                    data_frame = self.serial_handle.read(7)  # Read the rest of the frame
                    
                    # Verify checksum (optional but recommended)
                    checksum_calculated = (sum(header) + sum(data_frame[:-1])) & 0xFF
                    checksum_received = data_frame[-1]

                    if checksum_calculated == checksum_received:
                        dist_l = data_frame[0]
                        dist_h = data_frame[1]
                        distance_cm = dist_l + (dist_h << 8)
                        self.distancia_metros = distance_cm / 100.0
                        
                        msg = Range()
                        msg.radiation_type = Range.INFRARED  # TF-Luna is LiDAR (Light Detection and Ranging)
                        msg.field_of_view = 0.0349  # Approx 2 degrees for TF-Luna, in radians
                        msg.min_range = 0.20  # metros (TF-Luna typical min range)
                        msg.max_range = 8.0  # metros (TF-Luna typical max range, depends on model)
                        msg.range = self.distancia_metros
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "distance_sensor_link"  # Or your preferred frame_id
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'Publishing distance: {self.distancia_metros:.2f} m')
                    else:
                        self.get_logger().warn("TF-Luna checksum mismatch.")
                        # Clear buffer if checksum fails to avoid misaligned reads
                        self.serial_handle.reset_input_buffer()
                else:
                    # Not a TF-Luna header, try to resync by reading one byte
                    self.get_logger().debug("TF-Luna header mismatch, attempting to resync.")
                    self.serial_handle.reset_input_buffer()  # Clear buffer to help resync
            
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in sensor callback: {e}")

    def cleanup(self):
        self.get_logger().info("Cleaning up serial resources...")
        if self.serial_handle and self.serial_handle.is_open:
            try:
                self.serial_handle.close()
                self.get_logger().info("Serial port closed.")
            except serial.SerialException as e:
                self.get_logger().error(f"Error closing serial port: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('User interrupted node.')
    finally: 
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
