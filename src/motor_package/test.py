#!/usr/bin/env python3
"""
Script de testeo para motores del robot RoboBeach
Permite probar motores individuales y combinaciones sin ROS2
"""

import lgpio
import time
import threading

# Para Raspberry Pi 5, los GPIOs están típicamente en el chip 0
GPIO_CHIP = 0

class MotorTester:
    def __init__(self):
        # Servo pulse widths in microseconds (µs)
        self.SERVO_MIN_PULSE_US = 500    # 0 degrees
        self.SERVO_MAX_PULSE_US = 2500   # 180 degrees
        self.SERVO_MID_PULSE_US = 1500   # 90 degrees (approx)
        self.SERVO_OFF_PULSE_US = 0      # To stop sending pulses
        self.SERVO_FREQUENCY = 50        # Hz for servos

        # Motor PWM Frequency
        self.MOTOR_PWM_FREQUENCY = 1000  # Hz

        try:
            self.chip_handle = lgpio.gpiochip_open(GPIO_CHIP)
            print(f"GPIO chip {GPIO_CHIP} opened successfully")
        except lgpio.error as e:
            print(f"Failed to open GPIO chip {GPIO_CHIP}: {e}")
            print("Ensure lgpio daemon is running or you have permissions.")
            raise SystemExit(f"Fatal GPIO error: {e}")

        # Pines para motor 1 (Frontal izquierdo)
        self.FRONT_LEFT_A1 = 2  # in1
        self.FRONT_LEFT_A2 = 4  # in2
        self.FRONT_LEFT_PWMA = 3  # ENA

        # Pines para motor 2 (Frontal derecho)
        self.FRONT_RIGHT_B1 = 5  # in3
        self.FRONT_RIGHT_B2 = 7  # in4
        self.FRONT_RIGHT_PWMB = 6  # ENB

        # Pines para motor 3 (Trasero izquierdo)
        self.REAR_LEFT_A1 = 17  # A0
        self.REAR_LEFT_A2 = 27  # A1
        self.REAR_LEFT_PWMA = 9  # ENA_2

        # Pines para motor 4 (Trasero derecho)
        self.REAR_RIGHT_B1 = 22  # A2
        self.REAR_RIGHT_B2 = 23  # A3
        self.REAR_RIGHT_PWMB = 10  # ENB_2

        # Pines para el motor de la escoba
        self.BROOM_A = 13
        self.BROOM_B = 19
        self.BROOM_PWM = 20
        self.BROOM_STBY = 16

        # List of pins to be used as digital outputs
        self.digital_output_pins = [
            self.FRONT_LEFT_A1, self.FRONT_LEFT_A2,
            self.FRONT_RIGHT_B1, self.FRONT_RIGHT_B2,
            self.REAR_LEFT_A1, self.REAR_LEFT_A2,
            self.REAR_RIGHT_B1, self.REAR_RIGHT_B2,
            self.BROOM_A, self.BROOM_B, self.BROOM_STBY
        ]

        # Configurar todos los pines de salida digital
        for pin in self.digital_output_pins:
            try:
                lgpio.gpio_claim_output(self.chip_handle, pin)
                print(f"Pin {pin} configured as output")
            except lgpio.error as e:
                print(f"Failed to claim pin {pin} as output: {e}")

        # Activar el puente H de la escoba
        lgpio.gpio_write(self.chip_handle, self.BROOM_STBY, 1)

        # Inicializar PWM para motores de ruedas con 0% duty cycle
        lgpio.tx_pwm(self.chip_handle, self.FRONT_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, 0)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, 0)
        lgpio.tx_pwm(self.chip_handle, self.REAR_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, 0)
        lgpio.tx_pwm(self.chip_handle, self.REAR_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, 0)
        lgpio.tx_pwm(self.chip_handle, self.BROOM_PWM, self.MOTOR_PWM_FREQUENCY, 0)

        print("Motor tester initialized successfully!")

    def stop_all_motors(self):
        """Detiene todos los motores"""
        print("Deteniendo todos los motores...")
        
        # Motor 1 (Frontal izquierdo)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A1, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A2, 0)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_LEFT_PWMA, 0, 0)

        # Motor 2 (Frontal derecho)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B1, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B2, 0)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_RIGHT_PWMB, 0, 0)

        # Motor 3 (Trasero izquierdo)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A1, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A2, 0)
        lgpio.tx_pwm(self.chip_handle, self.REAR_LEFT_PWMA, 0, 0)

        # Motor 4 (Trasero derecho)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B1, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B2, 0)
        lgpio.tx_pwm(self.chip_handle, self.REAR_RIGHT_PWMB, 0, 0)

        # Motor escoba
        lgpio.gpio_write(self.chip_handle, self.BROOM_A, 0)
        lgpio.gpio_write(self.chip_handle, self.BROOM_B, 0)
        lgpio.tx_pwm(self.chip_handle, self.BROOM_PWM, 0, 0)

    def test_front_left_motor(self, duration=3, speed=50):
        """Prueba motor frontal izquierdo en ambas direcciones"""
        print(f"Probando motor frontal izquierdo por {duration} segundos cada dirección...")
        
        # Adelante
        print("  - Dirección: Adelante")
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A1, 1)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A2, 0)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, speed)
        time.sleep(duration)
        
        # Pausa
        lgpio.tx_pwm(self.chip_handle, self.FRONT_LEFT_PWMA, 0, 0)
        time.sleep(1)
        
        # Atrás
        print("  - Dirección: Atrás")
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A1, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A2, 1)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, speed)
        time.sleep(duration)
        
        # Detener
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A1, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A2, 0)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_LEFT_PWMA, 0, 0)

    def test_front_right_motor(self, duration=3, speed=50):
        """Prueba motor frontal derecho en ambas direcciones"""
        print(f"Probando motor frontal derecho por {duration} segundos cada dirección...")
        
        # Adelante
        print("  - Dirección: Adelante")
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B1, 1)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B2, 0)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, speed)
        time.sleep(duration)
        
        # Pausa
        lgpio.tx_pwm(self.chip_handle, self.FRONT_RIGHT_PWMB, 0, 0)
        time.sleep(1)
        
        # Atrás
        print("  - Dirección: Atrás")
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B1, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B2, 1)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, speed)
        time.sleep(duration)
        
        # Detener
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B1, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B2, 0)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_RIGHT_PWMB, 0, 0)

    def test_rear_left_motor(self, duration=3, speed=50):
        """Prueba motor trasero izquierdo en ambas direcciones"""
        print(f"Probando motor trasero izquierdo por {duration} segundos cada dirección...")
        
        # Adelante
        print("  - Dirección: Adelante")
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A1, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A2, 0)
        lgpio.tx_pwm(self.chip_handle, self.REAR_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, speed)
        time.sleep(duration)
        
        # Pausa
        lgpio.tx_pwm(self.chip_handle, self.REAR_LEFT_PWMA, 0, 0)
        time.sleep(1)
        
        # Atrás
        print("  - Dirección: Atrás")
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A1, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A2, 1)
        lgpio.tx_pwm(self.chip_handle, self.REAR_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, speed)
        time.sleep(duration)
        
        # Detener
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A1, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A2, 0)
        lgpio.tx_pwm(self.chip_handle, self.REAR_LEFT_PWMA, 0, 0)

    def test_rear_right_motor(self, duration=3, speed=50):
        """Prueba motor trasero derecho en ambas direcciones"""
        print(f"Probando motor trasero derecho por {duration} segundos cada dirección...")
        
        # Adelante (nota: este motor tiene la lógica invertida)
        print("  - Dirección: Adelante")
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B1, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B2, 1)
        lgpio.tx_pwm(self.chip_handle, self.REAR_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, speed)
        time.sleep(duration)
        
        # Pausa
        lgpio.tx_pwm(self.chip_handle, self.REAR_RIGHT_PWMB, 0, 0)
        time.sleep(1)
        
        # Atrás
        print("  - Dirección: Atrás")
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B1, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B2, 0)
        lgpio.tx_pwm(self.chip_handle, self.REAR_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, speed)
        time.sleep(duration)
        
        # Detener
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B1, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B2, 0)
        lgpio.tx_pwm(self.chip_handle, self.REAR_RIGHT_PWMB, 0, 0)

    def test_broom_motor(self, duration=5, speed=80):
        """Prueba motor de escoba en ambas direcciones"""
        print(f"Probando motor de escoba por {duration} segundos cada dirección...")
        
        # Dirección 1
        print("  - Dirección: 1")
        lgpio.gpio_write(self.chip_handle, self.BROOM_A, 1)
        lgpio.gpio_write(self.chip_handle, self.BROOM_B, 0)
        lgpio.tx_pwm(self.chip_handle, self.BROOM_PWM, self.MOTOR_PWM_FREQUENCY, speed)
        time.sleep(duration)
        
        # Pausa
        lgpio.tx_pwm(self.chip_handle, self.BROOM_PWM, 0, 0)
        time.sleep(1)
        
        # Dirección 2
        print("  - Dirección: 2")
        lgpio.gpio_write(self.chip_handle, self.BROOM_A, 0)
        lgpio.gpio_write(self.chip_handle, self.BROOM_B, 1)
        lgpio.tx_pwm(self.chip_handle, self.BROOM_PWM, self.MOTOR_PWM_FREQUENCY, speed)
        time.sleep(duration)
        
        # Detener
        lgpio.gpio_write(self.chip_handle, self.BROOM_A, 0)
        lgpio.gpio_write(self.chip_handle, self.BROOM_B, 0)
        lgpio.tx_pwm(self.chip_handle, self.BROOM_PWM, 0, 0)

    def test_all_forward(self, duration=5, speed=50):
        """Prueba todos los motores hacia adelante"""
        print(f"Probando todos los motores hacia adelante por {duration} segundos...")
        
        # Motor 1 (Frontal izquierdo)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A1, 1)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A2, 0)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, speed)

        # Motor 2 (Frontal derecho)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B1, 1)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B2, 0)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, speed)

        # Motor 3 (Trasero izquierdo)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A1, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A2, 0)
        lgpio.tx_pwm(self.chip_handle, self.REAR_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, speed)

        # Motor 4 (Trasero derecho) - lógica invertida
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B1, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B2, 1)
        lgpio.tx_pwm(self.chip_handle, self.REAR_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, speed)

        time.sleep(duration)
        self.stop_all_motors()

    def test_all_backward(self, duration=5, speed=50):
        """Prueba todos los motores hacia atrás"""
        print(f"Probando todos los motores hacia atrás por {duration} segundos...")
        
        # Motor 1 (Frontal izquierdo)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A1, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_LEFT_A2, 1)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, speed)

        # Motor 2 (Frontal derecho)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B1, 0)
        lgpio.gpio_write(self.chip_handle, self.FRONT_RIGHT_B2, 1)
        lgpio.tx_pwm(self.chip_handle, self.FRONT_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, speed)

        # Motor 3 (Trasero izquierdo)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A1, 0)
        lgpio.gpio_write(self.chip_handle, self.REAR_LEFT_A2, 1)
        lgpio.tx_pwm(self.chip_handle, self.REAR_LEFT_PWMA, self.MOTOR_PWM_FREQUENCY, speed)

        # Motor 4 (Trasero derecho) - lógica invertida
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B1, 1)
        lgpio.gpio_write(self.chip_handle, self.REAR_RIGHT_B2, 0)
        lgpio.tx_pwm(self.chip_handle, self.REAR_RIGHT_PWMB, self.MOTOR_PWM_FREQUENCY, speed)

        time.sleep(duration)
        self.stop_all_motors()

    def test_speed_variations(self, duration=2):
        """Prueba variaciones de velocidad en todos los motores"""
        print("Probando variaciones de velocidad...")
        speeds = [25, 50, 75, 100]
        
        for speed in speeds:
            print(f"  - Velocidad: {speed}%")
            self.test_all_forward(duration, speed)
            time.sleep(1)

    def cleanup(self):
        """Limpia recursos GPIO"""
        print("Limpiando recursos GPIO...")
        self.stop_all_motors()

        # Free claimed digital output GPIOs
        for pin in self.digital_output_pins:
            try:
                lgpio.gpio_write(self.chip_handle, pin, 0)
                lgpio.gpio_free(self.chip_handle, pin)
            except lgpio.error as e:
                print(f"Error freeing pin {pin}: {e}")

        # Close GPIO chip handle
        if hasattr(self, 'chip_handle'):
            try:
                lgpio.gpiochip_close(self.chip_handle)
                print("GPIO chip closed.")
            except lgpio.error as e:
                print(f"Error closing GPIO chip: {e}")

def main():
    print("=== MOTOR TESTER ROBOBEACH ===")
    print("Iniciando testeo de motores...")
    
    tester = MotorTester()
    
    try:
        while True:
            print("\n--- MENÚ DE PRUEBAS ---")
            print("1. Probar motor frontal izquierdo")
            print("2. Probar motor frontal derecho")
            print("3. Probar motor trasero izquierdo")
            print("4. Probar motor trasero derecho")
            print("5. Probar motor de escoba")
            print("6. Probar todos los motores adelante")
            print("7. Probar todos los motores atrás")
            print("8. Probar variaciones de velocidad")
            print("9. Detener todos los motores")
            print("0. Salir")
            
            choice = input("\nSelecciona una opción (0-9): ")
            
            if choice == '1':
                tester.test_front_left_motor()
            elif choice == '2':
                tester.test_front_right_motor()
            elif choice == '3':
                tester.test_rear_left_motor()
            elif choice == '4':
                tester.test_rear_right_motor()
            elif choice == '5':
                tester.test_broom_motor()
            elif choice == '6':
                tester.test_all_forward()
            elif choice == '7':
                tester.test_all_backward()
            elif choice == '8':
                tester.test_speed_variations()
            elif choice == '9':
                tester.stop_all_motors()
                print("Todos los motores detenidos.")
            elif choice == '0':
                break
            else:
                print("Opción no válida. Intenta de nuevo.")
            
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nInterrumpido por usuario...")
    finally:
        tester.cleanup()
        print("Testeo terminado.")

if __name__ == '__main__':
    main()