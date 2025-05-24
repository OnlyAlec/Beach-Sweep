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
import RPi.GPIO as GPIO
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.door = True 
        self.floor = True
        
        # Constantes para ángulos de servos
        self.SERVO_MIN_DUTY = 2.5    # 0 grados
        self.SERVO_MAX_DUTY = 12.5   # 180 grados
        self.SERVO_MID_DUTY = 7.5    # 90 grados
        self.SERVO_45_DUTY = 5.0     # 45 grados

        # Variables para control de espiral
        self.spiral_active = False
        self.spiral_right_speed = 0
        self.spiral_left_speed = 0
        self.spiral_speed_increment = 2  # Incremento de velocidad por iteración
        self.spiral_timer = None
        self.spiral_update_rate = 0.1  # Segundos entre actualizaciones de velocidad
        
        # Configuración de GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

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

        # Configurar todos los pines como salida
        GPIO.setup([
            self.FRONT_LEFT_A1, 
            self.FRONT_LEFT_A2, 
            self.FRONT_LEFT_PWMA,
            self.FRONT_RIGHT_B1, 
            self.FRONT_RIGHT_B2, 
            self.FRONT_RIGHT_PWMB,
            self.REAR_LEFT_A1, 
            self.REAR_LEFT_A2, 
            self.REAR_LEFT_PWMA,
            self.REAR_RIGHT_B1, 
            self.REAR_RIGHT_B2, 
            self.REAR_RIGHT_PWMB,
            self.FRONT_STBY, 
            self.REAR_STBY,
            self.BROOM_A, 
            self.BROOM_B, 
            self.BROOM_PWM,
            self.BROOM_STBY,
            self.SERVO_LEFT, 
            self.SERVO_RIGHT,
            self.SERVO_FLOOR
        ], GPIO.OUT)

        # Activar los puentes H
        GPIO.output(self.FRONT_STBY, GPIO.HIGH)
        GPIO.output(self.REAR_STBY, GPIO.HIGH)
        GPIO.output(self.BROOM_STBY, GPIO.HIGH)

        # PWM para motores de ruedas
        self.front_left_pwm = GPIO.PWM(self.FRONT_LEFT_PWMA, 1000)
        self.front_right_pwm = GPIO.PWM(self.FRONT_RIGHT_PWMB, 1000)
        self.rear_left_pwm = GPIO.PWM(self.REAR_LEFT_PWMA, 1000)
        self.rear_right_pwm = GPIO.PWM(self.REAR_RIGHT_PWMB, 1000)
        self.broom_pwm = GPIO.PWM(self.BROOM_PWM, 1000)

        # Iniciar PWM para motores de ruedas
        self.front_left_pwm.start(0)
        self.front_right_pwm.start(0)
        self.rear_left_pwm.start(0)
        self.rear_right_pwm.start(0)
        self.broom_pwm.start(0)

        # PWM para servos
        self.servo_left = GPIO.PWM(self.SERVO_LEFT, 50)
        self.servo_right = GPIO.PWM(self.SERVO_RIGHT, 50)
        self.servo_floor = GPIO.PWM(self.SERVO_FLOOR, 50)
        
        # Iniciar PWM para servos
        self.servo_left.start(0)
        self.servo_right.start(0)
        self.servo_floor.start(0)

        # Suscripción al tema de control
        self.subscription = self.create_subscription(String, 'robot/motor/commands', self.command_callback, 10)

        # Iniciar motor de escoba
        self.start_broom_motor()

    # Callback para recibir comandos
    def command_callback(self, msg):
        command = msg.data.upper()
        if command == "FORWARD":
            self.stop_spiral()  # Detener espiral si está activa
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
        GPIO.output(self.FRONT_LEFT_A1, GPIO.HIGH)
        GPIO.output(self.FRONT_LEFT_A2, GPIO.LOW)
        GPIO.output(self.FRONT_RIGHT_B1, GPIO.HIGH)
        GPIO.output(self.FRONT_RIGHT_B2, GPIO.LOW)
        GPIO.output(self.REAR_LEFT_A1, GPIO.HIGH)
        GPIO.output(self.REAR_LEFT_A2, GPIO.LOW)
        GPIO.output(self.REAR_RIGHT_B1, GPIO.HIGH)
        GPIO.output(self.REAR_RIGHT_B2, GPIO.LOW)

        self.set_motor_speed(70)

    # Retroceder
    def move_backward(self):
        self.get_logger().info("Moviendo hacia atrás")
        GPIO.output(self.FRONT_LEFT_A1, GPIO.LOW)
        GPIO.output(self.FRONT_LEFT_A2, GPIO.HIGH)
        GPIO.output(self.FRONT_RIGHT_B1, GPIO.LOW)
        GPIO.output(self.FRONT_RIGHT_B2, GPIO.HIGH)
        GPIO.output(self.REAR_LEFT_A1, GPIO.LOW)
        GPIO.output(self.REAR_LEFT_A2, GPIO.HIGH)
        GPIO.output(self.REAR_RIGHT_B1, GPIO.LOW)
        GPIO.output(self.REAR_RIGHT_B2, GPIO.HIGH)

        self.set_motor_speed(70)

    # Girar a la izquierda
    def turn_left(self):
        self.get_logger().info("Girando a la izquierda")
        GPIO.output(self.FRONT_LEFT_A1, GPIO.LOW)
        GPIO.output(self.FRONT_LEFT_A2, GPIO.HIGH)
        GPIO.output(self.FRONT_RIGHT_B1, GPIO.HIGH)
        GPIO.output(self.FRONT_RIGHT_B2, GPIO.LOW)
        GPIO.output(self.REAR_LEFT_A1, GPIO.LOW)
        GPIO.output(self.REAR_LEFT_A2, GPIO.HIGH)
        GPIO.output(self.REAR_RIGHT_B1, GPIO.HIGH)
        GPIO.output(self.REAR_RIGHT_B2, GPIO.LOW)

        self.set_motor_speed(50)

    # Girar a la derecha
    def turn_right(self):
        self.get_logger().info("Girando a la derecha")
        GPIO.output(self.FRONT_LEFT_A1, GPIO.HIGH)
        GPIO.output(self.FRONT_LEFT_A2, GPIO.LOW)
        GPIO.output(self.FRONT_RIGHT_B1, GPIO.LOW)
        GPIO.output(self.FRONT_RIGHT_B2, GPIO.HIGH)
        GPIO.output(self.REAR_LEFT_A1, GPIO.HIGH)
        GPIO.output(self.REAR_LEFT_A2, GPIO.LOW)
        GPIO.output(self.REAR_RIGHT_B1, GPIO.LOW)
        GPIO.output(self.REAR_RIGHT_B2, GPIO.HIGH)

        self.set_motor_speed(50)

    # Detener motores
    def stop_motors(self):
        self.set_motor_speed(0)

    # Establecer velocidad de los motores
    def set_motor_speed(self, duty):
        self.front_left_pwm.ChangeDutyCycle(duty)
        self.front_right_pwm.ChangeDutyCycle(duty)
        self.rear_left_pwm.ChangeDutyCycle(duty)
        self.rear_right_pwm.ChangeDutyCycle(duty)

    # Abrir compuerta
    def open_door(self):
        """Abre la compuerta de entrada a 180 grados."""
        self.door = False
        self.servo_left.ChangeDutyCycle(self.SERVO_MAX_DUTY)  # 180 grados
        self.servo_right.ChangeDutyCycle(self.SERVO_MAX_DUTY)
        time.sleep(0.5)
        self.servo_left.ChangeDutyCycle(0)
        self.servo_right.ChangeDutyCycle(0)

    # Cerrar compuerta
    def close_door(self):
        """Cierra la compuerta de entrada a 0 grados."""
        self.door = True
        self.servo_left.ChangeDutyCycle(self.SERVO_MIN_DUTY)  # 0 grados
        self.servo_right.ChangeDutyCycle(self.SERVO_MIN_DUTY)
        time.sleep(0.5)
        self.servo_left.ChangeDutyCycle(0)
        self.servo_right.ChangeDutyCycle(0)

    def incline_floor(self):
        """Inclina el piso de almacenamiento a 45 grados para descargar el contenido."""
        self.floor = False
        self.servo_floor.ChangeDutyCycle(self.SERVO_45_DUTY)  # 45 grados
        time.sleep(0.5)  # Esperar a que el servo alcance la posición
        self.servo_floor.ChangeDutyCycle(0)  # Detener el PWM para evitar vibraciones
        self.get_logger().info("Piso inclinado a 45 grados")

    def deincline_floor(self):
        """Devuelve el piso de almacenamiento a la posición horizontal."""
        self.floor = True
        self.servo_floor.ChangeDutyCycle(self.SERVO_MIN_DUTY)  # 0 grados
        time.sleep(0.5)
        self.servo_floor.ChangeDutyCycle(0)
        self.get_logger().info("Piso devuelto a posición horizontal")

    def start_broom_motor(self):
        """Inicia el motor de la escoba."""
        GPIO.output(self.BROOM_A, GPIO.HIGH)
        GPIO.output(self.BROOM_B, GPIO.LOW)
        self.broom_pwm.ChangeDutyCycle(80)
        self.get_logger().info("Motor de escoba iniciado")

    def cleanup(self):
        self.stop_spiral()  # Asegurar que la espiral se detenga
        self.front_left_pwm.stop()
        self.front_right_pwm.stop()
        self.rear_left_pwm.stop()
        self.rear_right_pwm.stop()
        self.broom_pwm.stop()
        self.servo_left.stop()
        self.servo_right.stop()
        GPIO.cleanup()

    def start_spiral(self):
        """Inicia el movimiento en espiral desde el centro hacia afuera."""
        if not self.spiral_active:
            self.spiral_active = True
            self.spiral_right_speed = 30  # Velocidad inicial derecha
            self.spiral_left_speed = 10   # Velocidad inicial izquierda
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

        # Incrementar velocidades
        self.spiral_right_speed = min(100, self.spiral_right_speed + self.spiral_speed_increment)
        self.spiral_left_speed = min(100, self.spiral_left_speed + self.spiral_speed_increment)

        # Configurar dirección de los motores para movimiento hacia adelante
        GPIO.output(self.FRONT_LEFT_A1, GPIO.HIGH)
        GPIO.output(self.FRONT_LEFT_A2, GPIO.LOW)
        GPIO.output(self.FRONT_RIGHT_B1, GPIO.HIGH)
        GPIO.output(self.FRONT_RIGHT_B2, GPIO.LOW)
        GPIO.output(self.REAR_LEFT_A1, GPIO.HIGH)
        GPIO.output(self.REAR_LEFT_A2, GPIO.LOW)
        GPIO.output(self.REAR_RIGHT_B1, GPIO.HIGH)
        GPIO.output(self.REAR_RIGHT_B2, GPIO.LOW)

        # Aplicar velocidades diferentes para crear el efecto espiral
        self.front_left_pwm.ChangeDutyCycle(self.spiral_left_speed)
        self.front_right_pwm.ChangeDutyCycle(self.spiral_right_speed)
        self.rear_left_pwm.ChangeDutyCycle(self.spiral_left_speed)
        self.rear_right_pwm.ChangeDutyCycle(self.spiral_right_speed)

        # Programar la siguiente actualización
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
