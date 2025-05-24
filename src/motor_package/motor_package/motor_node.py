"""
Módulo Motor Controller para sistema de recolección de basura con ROS2

Responsabilidades:
1. Controlar motores de tracción de 4 ruedas independientes
2. Manejar servomotores para compuertas y mecanismos de recolección
3. Controlar motor de escoba para limpieza activa
4. Ejecutar comandos de movimiento (adelante, atrás, giros, espiral)
5. Gestionar mecanismos de recolección y descarga de basura
6. Mantener control de GPIO para Raspberry Pi

Tópicos:
- Suscrito a: /robot/motor/commands (std_msgs/String)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lgpio
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For Raspberry Pi 5, GPIOs are typically on chip 4
GPIO_CHIP = 0

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.door = True
        self.floor = True

        # Servo pulse widths in microseconds (µs)
        self.SERVO_MIN_PULSE_US = 500    # 0 degrees
        self.SERVO_MAX_PULSE_US = 2500   # 180 degrees
        self.SERVO_MID_PULSE_US = 1500   # 90 degrees (approx)
        self.SERVO_45_PULSE_US = 1000    # 45 degrees (approx)
        self.SERVO_OFF_PULSE_US = 0      # To stop sending pulses
        self.SERVO_FREQUENCY = 50        # Hz for servos

        # Motor PWM Frequency
        self.MOTOR_PWM_FREQUENCY = 1000  # Hz

        # Variables para control de espiral
        self.spiral_active = False
        self.spiral_right_speed = 0
        self.spiral_left_speed = 0
        self.spiral_speed_increment = 2  # Incremento de velocidad por iteración
        self.spiral_timer = None
        self.spiral_update_rate = 0.1  # Segundos entre actualizaciones de velocidad

        try:
            self.chip_handle = lgpio.gpiochip_open(GPIO_CHIP)
        except lgpio.error as e:
            self.get_logger().error(f"Failed to open GPIO chip {GPIO_CHIP}: {e}")
            self.get_logger().error("Ensure lgpio daemon is running or you have permissions.")
            self.get_logger().error("For Pi 5, ensure you are using the correct chip (e.g., 4). Check with 'gpioinfo'.")
            rclpy.shutdown()
            raise SystemExit(f"Fatal GPIO error: {e}")

        # Pines para motor rueda frontal izquierda
        self.FRONT_LEFT_A1 = 6
        self.FRONT_LEFT_A2 = 5
        self.FRONT_LEFT_PWMA = 12

        # Pines para motor rueda frontal derecha
        self.FRONT_RIGHT_B1 = 16
        self.FRONT_RIGHT_B2 = 20
        self.FRONT_RIGHT_PWMB = 19

        # Pines para motor rueda trasera izquierda
        self.REAR_LEFT_A1 = 27
        self.REAR_LEFT_A2 = 17
        self.REAR_LEFT_PWMA = 18

        # Pines para motor rueda trasera derecha
        self.REAR_RIGHT_B1 = 22
        self.REAR_RIGHT_B2 = 23
        self.REAR_RIGHT_PWMB = 13

        # Pines STBY de los puentes H
        self.FRONT_STBY = 24
        self.REAR_STBY = 4

        # Pines para los servos
        self.SERVO_LEFT = 21
        self.SERVO_RIGHT = 25
        self.SERVO_FLOOR = 26

        # Pines para el motor de la escoba
        self.BROOM_A = 9
        self.BROOM_B = 10
        self.BROOM_PWM = 11
        self.BROOM_STBY = 8

        # List of pins to be used as digital outputs
        self.digital_output_pins = [
            self.FRONT_LEFT_A1, self.FRONT_LEFT_A2,
            self.FRONT_RIGHT_B1, self.FRONT_RIGHT_B2,
            self.REAR_LEFT_A1, self.REAR_LEFT_A2,
            self.REAR_RIGHT_B1, self.REAR_RIGHT_B2,
            self.FRONT_STBY, self.REAR_STBY,
            self.BROOM_A, self.BROOM_B, self.BROOM_STBY
        ]

        # Configurar todos los pines de salida digital
        for pin in self.digital_output_pins:
            try:
                lgpio.gpio_claim_output(self.chip_handle, pin)
            except lgpio.error as e:
                self.get_logger().error(f"Failed to claim pin {pin} as output: {e}")

        # Activar los puentes H (set STBY pins HIGH)
        lgpio.gpio_write(self.chip_handle, self.FRONT_STBY, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_STBY, 1)
        lgpio.gpio_write(self.chip_handle, self.BROOM_STBY, 1)

        # PWM para motores de ruedas - initialized to 0% duty cycle
        lgpio.tx_pwm(self.chip_handle, self.FRONT_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, 0)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, 0)
        lgpio.tx_pwm(self.chip_handle, self.REAR_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, 0)
        lgpio.tx_pwm(self.chip_handle, self.REAR_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, 0)
        lgpio.tx_pwm(self.chip_handle, self.BROOM_PWM, self.MOTOR_PWM_FREQUENCY, 0)

        # PWM para servos - initialized to minimum pulse width (0 degrees)
        try:
            lgpio.tx_servo(self.chip_handle, self.SERVO_LEFT, self.SERVO_MIN_PULSE_US, self.SERVO_FREQUENCY)
            lgpio.tx_servo(self.chip_handle, self.SERVO_RIGHT, self.SERVO_MIN_PULSE_US, self.SERVO_FREQUENCY)
            lgpio.tx_servo(self.chip_handle, self.SERVO_FLOOR, self.SERVO_MIN_PULSE_US, self.SERVO_FREQUENCY)
        except lgpio.error as e:
            self.get_logger().error(f"Failed to initialize servos: {e}")
            # Optionally, re-raise or handle more gracefully if servo init failure is critical
            # For now, just log and continue, as chip is already open.

        # Usar QoS RELIABLE para asegurar la recepción de comandos
        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(String, 'robot/motor/commands', self.command_callback, qos_reliable)

        # Iniciar motor de escoba
        self.start_broom_motor()

    # Callback para recibir comandos
    def command_callback(self, msg):
        command = msg.data.upper()
        if command == "FORWARD":
            self.stop_spiral()
            self.move_forward()
        elif command == "BACKWARD":
            self.stop_spiral()
            self.move_backward()
        elif command == "LEFT":
            self.stop_spiral()
            self.turn_left()
        elif command == "RIGHT":
            self.stop_spiral()
            self.turn_right()
        elif command == "STOP":
            self.stop_spiral()
            self.stop_motors()
        elif command == "SPIRAL":
            self.start_spiral()
        elif command == "OPEN_DOOR" and self.door:
            self.stop_spiral()
            self.open_door()
        elif command == "CLOSE_DOOR" and not self.door:
            self.stop_spiral()
            self.close_door()
        elif command == "OPEN_DOOR" and not self.door:
            self.get_logger().info("Door Opened")
        elif command == "CLOSE_DOOR" and self.door:
            self.get_logger().info("Door Closed")
        elif command == "INCLINE" and self.floor:
            self.stop_spiral()
            self.incline_floor()
        elif command == "DEINCLINE" and not self.floor:
            self.stop_spiral()
            self.deincline_floor()
        elif command == "INCLINE" and not self.floor:
            self.get_logger().info("Floor Inclined")
        elif command == "DEINCLINE" and self.floor:
            self.get_logger().info("Floor Deinclined")
        else:
            self.get_logger().info(f"Comando desconocido: {command}")

    # Métodos para controlar los motores
    # Avanzar
    def move_forward(self):
        self.get_logger().info("Moviendo hacia adelante")
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A1, 1)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A2, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B1, 1)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B2, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A1, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A2, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B1, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B2, 0)

        self.set_motor_speed(70)

    # Retroceder
    def move_backward(self):
        self.get_logger().info("Moviendo hacia atrás")
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A1, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A2, 1)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B1, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B2, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A1, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A2, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B1, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B2, 1)

        self.set_motor_speed(70)

    # Girar a la izquierda
    def turn_left(self):
        self.get_logger().info("Girando a la izquierda")
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A1, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A2, 1)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B1, 1)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B2, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A1, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A2, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B1, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B2, 0)

        self.set_motor_speed(50)

    # Girar a la derecha
    def turn_right(self):
        self.get_logger().info("Girando a la derecha")
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A1, 1)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A2, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B1, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B2, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A1, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A2, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B1, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B2, 1)

        self.set_motor_speed(50)

    # Detener motores
    def stop_motors(self):
        self.set_motor_speed(0)

    # Establecer velocidad de los motores (duty is 0-100)
    def set_motor_speed(self, duty_percent):
        lgpio.tx_pwm(self.chip_handle, self.FRONT_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, duty_percent)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, duty_percent)
        lgpio.tx_pwm(self.chip_handle, self.REAR_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, duty_percent)
        lgpio.tx_pwm(self.chip_handle, self.REAR_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, duty_percent)

    # Abrir compuerta
    def open_door(self):
        """Abre la compuerta de entrada a 180 grados."""
        self.door = False
        lgpio.tx_servo(self.chip_handle, self.SERVO_LEFT, self.SERVO_MAX_PULSE_US, self.SERVO_FREQUENCY)
        lgpio.tx_servo(self.chip_handle, self.SERVO_RIGHT, self.SERVO_MAX_PULSE_US, self.SERVO_FREQUENCY)
        time.sleep(0.5)
        lgpio.tx_servo(self.chip_handle, self.SERVO_LEFT, self.SERVO_OFF_PULSE_US, self.SERVO_FREQUENCY)
        lgpio.tx_servo(self.chip_handle, self.SERVO_RIGHT, self.SERVO_OFF_PULSE_US, self.SERVO_FREQUENCY)

    # Cerrar compuerta
    def close_door(self):
        """Cierra la compuerta de entrada a 0 grados."""
        self.door = True
        lgpio.tx_servo(self.chip_handle, self.SERVO_LEFT, self.SERVO_MIN_PULSE_US, self.SERVO_FREQUENCY)
        lgpio.tx_servo(self.chip_handle, self.SERVO_RIGHT, self.SERVO_MIN_PULSE_US, self.SERVO_FREQUENCY)
        time.sleep(0.5)
        lgpio.tx_servo(self.chip_handle, self.SERVO_LEFT, self.SERVO_OFF_PULSE_US, self.SERVO_FREQUENCY)
        lgpio.tx_servo(self.chip_handle, self.SERVO_RIGHT, self.SERVO_OFF_PULSE_US, self.SERVO_FREQUENCY)

    def incline_floor(self):
        """Inclina el piso de almacenamiento a 45 grados para descargar el contenido."""
        self.floor = False
        lgpio.tx_servo(self.chip_handle, self.SERVO_FLOOR, self.SERVO_45_PULSE_US, self.SERVO_FREQUENCY)
        time.sleep(0.5)
        lgpio.tx_servo(self.chip_handle, self.SERVO_FLOOR, self.SERVO_OFF_PULSE_US, self.SERVO_FREQUENCY)
        self.get_logger().info("Piso inclinado a 45 grados")

    def deincline_floor(self):
        """Devuelve el piso de almacenamiento a la posición horizontal."""
        self.floor = True
        lgpio.tx_servo(self.chip_handle, self.SERVO_FLOOR, self.SERVO_MIN_PULSE_US, self.SERVO_FREQUENCY)
        time.sleep(0.5)
        lgpio.tx_servo(self.chip_handle, self.SERVO_FLOOR, self.SERVO_OFF_PULSE_US, self.SERVO_FREQUENCY)
        self.get_logger().info("Piso devuelto a posición horizontal")

    def start_broom_motor(self):
        """Inicia el motor de la escoba."""
        lgpio.gpio_write(self.chip_handle, self.BROOM_A, 1)
        lgpio.gpio_write(self.chip_handle, self.BROOM_B, 0)
        lgpio.tx_pwm(self.chip_handle, self.BROOM_PWM, self.MOTOR_PWM_FREQUENCY, 80)
        self.get_logger().info("Motor de escoba iniciado")

    def cleanup(self):
        self.get_logger().info("Cleaning up GPIO resources...")
        self.stop_spiral()

        # Stop all PWM signals
        motor_pwm_pins = [
            self.FRONT_LEFT_PWMA, self.FRONT_RIGHT_PWMB,
            self.REAR_LEFT_PWMA, self.REAR_RIGHT_PWMB,
            self.BROOM_PWM
        ]
        for pin in motor_pwm_pins:
            try:
                lgpio.tx_pwm(self.chip_handle, pin, 0, 0)
            except lgpio.error as e:
                self.get_logger().warn(f"Error stopping PWM on pin {pin}: {e}")

        servo_pins = [self.SERVO_LEFT, self.SERVO_RIGHT, self.SERVO_FLOOR]
        for pin in servo_pins:
            try:
                lgpio.tx_servo(self.chip_handle, pin, 0, 0)
            except lgpio.error as e:
                self.get_logger().warn(f"Error stopping servo on pin {pin}: {e}")
        
        # Free claimed digital output GPIOs
        for pin in self.digital_output_pins:
            try:
                lgpio.gpio_write(self.chip_handle, pin, 0)
                lgpio.gpio_free(self.chip_handle, pin)
            except lgpio.error as e:
                self.get_logger().warn(f"Error freeing pin {pin}: {e}")

        # Close GPIO chip handle
        if hasattr(self, 'chip_handle'):
            try:
                lgpio.gpiochip_close(self.chip_handle)
                self.get_logger().info("GPIO chip closed.")
            except lgpio.error as e:
                self.get_logger().error(f"Error closing GPIO chip: {e}")

    def start_spiral(self):
        """Inicia el movimiento en espiral desde el centro hacia afuera."""
        if not self.spiral_active:
            self.spiral_active = True
            self.spiral_right_speed = 30
            self.spiral_left_speed = 10
            self.get_logger().info("Iniciando movimiento en espiral")
            self.update_spiral()

    def stop_spiral(self):
        """Detiene el movimiento en espiral y reinicia sus variables."""
        if self.spiral_active:
            self.spiral_active = False
            if self.spiral_timer is not None:
                self.spiral_timer.cancel()
                self.spiral_timer = None
            self.spiral_right_speed = 0
            self.spiral_left_speed = 0
            self.stop_motors()
            self.get_logger().info("Deteniendo movimiento en espiral")

    def update_spiral(self):
        """Actualiza las velocidades del movimiento en espiral."""
        if not self.spiral_active:
            return

        self.spiral_right_speed = min(100, self.spiral_right_speed + self.spiral_speed_increment)
        self.spiral_left_speed = min(100, self.spiral_left_speed + self.spiral_speed_increment)

        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A1, 1)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A2, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B1, 1)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B2, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A1, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A2, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B1, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B2, 0)

        lgpio.tx_pwm(self.chip_handle, self.FRONT_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, self.spiral_left_speed)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, self.spiral_right_speed)
        lgpio.tx_pwm(self.chip_handle, self.REAR_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, self.spiral_left_speed)
        lgpio.tx_pwm(self.chip_handle, self.REAR_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, self.spiral_right_speed)

        if self.spiral_active:
            self.spiral_timer = self.create_timer(self.spiral_update_rate, self.update_spiral)

def main(args=None):
    rclpy.init(args=args)
    controller = MotorController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('🛑 Nodo interrumpido por el usuario.')
    finally:
        controller.cleanup()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
