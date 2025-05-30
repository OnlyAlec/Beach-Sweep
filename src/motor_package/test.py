#!/usr/bin/env python3
"""
Archivo de prueba de pines para verificar motores y servos usando gpiozero
Prueba individual de cada motor y servo por 3 segundos cada uno
No requiere ROS2, solo gpiozero

Uso:
    python3 test_motor_pins_gpiozero.py

IMPORTANTE: Ejecutar con permisos adecuados para GPIO
Instalar gpiozero: sudo apt install python3-gpiozero (ya viene preinstalado en Raspberry Pi OS)
"""

from gpiozero import LED, PWMOutputDevice, Servo, Device
from gpiozero.pins.lgpio import LGPIOFactory
import time
import sys
import signal
import threading
import queue
import queue

# Usar lgpio como backend (más compatible con Pi 5)
Device.pin_factory = LGPIOFactory()

class MotorPinTester:
    def __init__(self):
        print("🔧 Iniciando prueba de pines de motores con gpiozero...")
        
        # Definición de pines - Motor rueda frontal izquierda
        self.FRONT_LEFT_A1 = 16
        self.FRONT_LEFT_A2 = 20
        self.FRONT_LEFT_PWMA = 19

        # Motor rueda frontal derecha
        self.FRONT_RIGHT_B1 = 6
        self.FRONT_RIGHT_B2 = 5
        self.FRONT_RIGHT_PWMB = 12

        # Motor rueda trasera izquierda
        self.REAR_LEFT_A1 = 22
        self.REAR_LEFT_A2 = 23
        self.REAR_LEFT_PWMA = 13

        # Motor rueda trasera derecha
        self.REAR_RIGHT_B1 = 17
        self.REAR_RIGHT_B2 = 27
        self.REAR_RIGHT_PWMB = 18

        # Pines para los servos
        self.SERVO_LEFT = 25
        self.SERVO_RIGHT = 21
        self.SERVO_FLOOR = 26

        # Pines para el motor de la escoba
        self.BROOM_A = 9
        self.BROOM_B = 10
        self.BROOM_PWM = 11
        self.BROOM_STBY = 8

        # Diccionario para almacenar objetos de control
        self.digital_pins = {}
        self.pwm_pins = {}
        self.servo_objects = {}

        # Variables para control con threading
        self.key_queue = queue.Queue()
        self.input_thread = None
        self.stop_input = False

        self.setup_pins()

    def setup_pins(self):
        """Configurar todos los pines necesarios"""
        print("🔧 Configurando pines...")
        
        try:
            # Lista de pines digitales de control
            digital_pin_list = [
                self.FRONT_LEFT_A1, self.FRONT_LEFT_A2,
                self.FRONT_RIGHT_B1, self.FRONT_RIGHT_B2,
                self.REAR_LEFT_A1, self.REAR_LEFT_A2,
                self.REAR_RIGHT_B1, self.REAR_RIGHT_B2,
                self.BROOM_A, self.BROOM_B, self.BROOM_STBY
            ]
            
            # Configurar pines digitales
            for pin in digital_pin_list:
                self.digital_pins[pin] = LED(pin)
                self.digital_pins[pin].off()  # Inicializar en LOW
            
            # Lista de pines PWM para motores
            motor_pwm_list = [
                self.FRONT_LEFT_PWMA, self.FRONT_RIGHT_PWMB,
                self.REAR_LEFT_PWMA, self.REAR_RIGHT_PWMB,
                self.BROOM_PWM
            ]
            
            # Configurar pines PWM para motores
            for pin in motor_pwm_list:
                self.pwm_pins[pin] = PWMOutputDevice(pin, frequency=1000)
                self.pwm_pins[pin].value = 0  # Iniciar con 0% duty cycle
            
            # Configurar servos
            servo_pin_list = [self.SERVO_LEFT, self.SERVO_RIGHT, self.SERVO_FLOOR]
            for pin in servo_pin_list:
                # Servo con rango completo (-1 a 1)
                self.servo_objects[pin] = Servo(pin, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
                self.servo_objects[pin].value = None  # Servo en posición neutral/off

            # Activar los puentes H (set STBY pins HIGH)
            self.digital_pins[self.BROOM_STBY].on()
            
            print("✅ Puentes H activados")
            print("✅ Pines configurados correctamente")
            
        except Exception as e:
            print(f"❌ Error configurando pines: {e}")
            self.cleanup()
            sys.exit(1)

    def test_motor(self, name, a1_pin, a2_pin, pwm_pin, speed=0.5):
        """Prueba un motor individual"""
        print(f"🔄 Probando {name} - Adelante por 3 segundos...")
        
        # Dirección adelante
        self.digital_pins[a1_pin].on()
        self.digital_pins[a2_pin].off()
        self.pwm_pins[pwm_pin].value = speed  # speed entre 0.0 y 1.0
        time.sleep(3)
        
        # Parar
        self.pwm_pins[pwm_pin].value = 0
        time.sleep(0.5)
        
        print(f"🔄 Probando {name} - Atrás por 3 segundos...")
        
        # Dirección atrás
        self.digital_pins[a1_pin].off()
        self.digital_pins[a2_pin].on()
        self.pwm_pins[pwm_pin].value = speed
        time.sleep(3)
        
        # Parar
        self.pwm_pins[pwm_pin].value = 0
        self.digital_pins[a1_pin].off()
        self.digital_pins[a2_pin].off()
        time.sleep(0.5)
        
        print(f"✅ {name} completado")

    def test_servo(self, name, servo_pin):
        """Prueba un servo individual"""
        print(f"🔄 Probando {name} - Movimiento completo por 3 segundos...")
        
        try:
            # Posición -1 (0 grados)
            print(f"   → Posición 0° (min)")
            self.servo_objects[servo_pin].value = -1
            time.sleep(1)
            
            # Posición 0 (90 grados)
            print(f"   → Posición 90° (medio)")
            self.servo_objects[servo_pin].value = 0
            time.sleep(1)
            
            # Posición 1 (180 grados)
            print(f"   → Posición 180° (max)")
            self.servo_objects[servo_pin].value = 1
            time.sleep(1)
            
            # Detener señal PWM
            print(f"   → Deteniendo servo")
            self.servo_objects[servo_pin].value = None
            
        except Exception as e:
            print(f"❌ Error controlando servo {name} en pin {servo_pin}: {e}")
        
        print(f"✅ {name} completado")

    def input_thread_function(self):
        """Función para el hilo de entrada de teclado"""
        while not self.stop_input:
            try:
                key = input()  # Esperar entrada de teclado
                if key.lower() == 'esc' or key == '\x1b':
                    self.key_queue.put('ESC')
                else:
                    self.key_queue.put(key)
            except:
                break

    def start_input_thread(self):
        """Iniciar el hilo de entrada de teclado"""
        self.stop_input = False
        self.input_thread = threading.Thread(target=self.input_thread_function, daemon=True)
        self.input_thread.start()

    def stop_input_thread(self):
        """Detener el hilo de entrada de teclado"""
        self.stop_input = True
        if self.input_thread and self.input_thread.is_alive():
            self.input_thread.join(timeout=1)

    def get_key_pressed(self):
        """Obtener tecla presionada de la cola (no bloqueante)"""
        try:
            return self.key_queue.get_nowait()
        except queue.Empty:
            return None

    def test_individual_motor_continuous(self, name, a1_pin, a2_pin, pwm_pin, speed=0.5):
        """Prueba un motor individual de forma continua hasta presionar ESC"""
        print(f"\n🔄 Probando {name} CONTINUAMENTE")
        print("=" * 50)
        print("⚠️  El motor girará continuamente")
        print("🔧 Escribe comandos y presiona ENTER")
        print("📋 Comandos: 1=Adelante, 2=Atrás, 3=Parar, esc=Salir")
        print()
        
        self.start_input_thread()
        
        try:
            direction = 0  # 0=parado, 1=adelante, 2=atrás
            
            while True:
                # Verificar si se presionó una tecla
                key = self.get_key_pressed()
                
                if key:
                    if key.upper() == 'ESC':
                        print(f"\n🛑 Deteniendo {name} y regresando al menú...")
                        break
                    elif key == '1':
                        print(f"➡️  {name} - Adelante")
                        direction = 1
                        self.digital_pins[a1_pin].on()
                        self.digital_pins[a2_pin].off()
                        self.pwm_pins[pwm_pin].value = speed
                    elif key == '2':
                        print(f"⬅️  {name} - Atrás")
                        direction = 2
                        self.digital_pins[a1_pin].off()
                        self.digital_pins[a2_pin].on()
                        self.pwm_pins[pwm_pin].value = speed
                    elif key == '3':
                        print(f"⏹️  {name} - Parado")
                        direction = 0
                        self.pwm_pins[pwm_pin].value = 0
                        self.digital_pins[a1_pin].off()
                        self.digital_pins[a2_pin].off()
                
                # Esperar un poco antes de verificar de nuevo
                time.sleep(0.1)
                
        except Exception as e:
            print(f"❌ Error controlando {name}: {e}")
        finally:
            self.stop_input_thread()
            
            # Detener motor al salir
            self.pwm_pins[pwm_pin].value = 0
            self.digital_pins[a1_pin].off()
            self.digital_pins[a2_pin].off()
            print(f"✅ {name} detenido")

    def test_individual_servo_continuous(self, name, servo_pin):
        """Prueba un servo individual de forma continua hasta presionar ESC"""
        print(f"\n🔄 Probando {name} CONTINUAMENTE")
        print("=" * 50)
        print("⚠️  El servo se moverá según los comandos")
        print("🔧 Escribe comandos y presiona ENTER")
        print("📋 Comandos: 1=0°, 2=45°, 3=90°, 4=135°, 5=180°, 0=Parar, esc=Salir")
        print()
        
        self.start_input_thread()
        
        try:
            while True:
                # Verificar si se presionó una tecla
                key = self.get_key_pressed()
                
                if key:
                    if key.upper() == 'ESC':
                        print(f"\n🛑 Deteniendo {name} y regresando al menú...")
                        break
                    elif key == '1':
                        print(f"📐 {name} - 0° (min)")
                        self.servo_objects[servo_pin].value = -1
                    elif key == '2':
                        print(f"📐 {name} - 45°")
                        self.servo_objects[servo_pin].value = -0.5
                    elif key == '3':
                        print(f"📐 {name} - 90° (medio)")
                        self.servo_objects[servo_pin].value = 0
                    elif key == '4':
                        print(f"📐 {name} - 135°")
                        self.servo_objects[servo_pin].value = 0.5
                    elif key == '5':
                        print(f"📐 {name} - 180° (max)")
                        self.servo_objects[servo_pin].value = 1
                    elif key == '0':
                        print(f"⏹️  {name} - Parado")
                        self.servo_objects[servo_pin].value = None
                
                time.sleep(0.1)
                
        except Exception as e:
            print(f"❌ Error controlando {name}: {e}")
        finally:
            self.stop_input_thread()
            
            # Detener servo al salir
            self.servo_objects[servo_pin].value = None
            print(f"✅ {name} detenido")

    def individual_motor_menu(self):
        """Menú para seleccionar motor individual"""
        while True:
            print("\n🎯 CONTROL INDIVIDUAL DE MOTORES")
            print("=" * 50)
            print("Selecciona el motor a controlar:")
            print()
            print("🚗 MOTORES DE TRACCIÓN:")
            print("1. Motor Frontal Izquierdo")
            print("2. Motor Frontal Derecho") 
            print("3. Motor Trasero Izquierdo")
            print("4. Motor Trasero Derecho")
            print()
            print("🧹 MOTOR DE ESCOBA:")
            print("5. Motor de Escoba")
            print()
            print("🚪 SERVOMOTORES:")
            print("6. Servo Compuerta Izquierda")
            print("7. Servo Compuerta Derecha")
            print("8. Servo Piso")
            print()
            print("0. Regresar al menú principal")
            
            choice = input("\nSelecciona una opción (0-8): ").strip()
            
            if choice == "1":
                self.test_individual_motor_continuous("Motor Frontal Izquierdo", 
                    self.FRONT_LEFT_A1, self.FRONT_LEFT_A2, self.FRONT_LEFT_PWMA)
            elif choice == "2":
                self.test_individual_motor_continuous("Motor Frontal Derecho", 
                    self.FRONT_RIGHT_B1, self.FRONT_RIGHT_B2, self.FRONT_RIGHT_PWMB)
            elif choice == "3":
                self.test_individual_motor_continuous("Motor Trasero Izquierdo", 
                    self.REAR_LEFT_A1, self.REAR_LEFT_A2, self.REAR_LEFT_PWMA)
            elif choice == "4":
                self.test_individual_motor_continuous("Motor Trasero Derecho", 
                    self.REAR_RIGHT_B1, self.REAR_RIGHT_B2, self.REAR_RIGHT_PWMB)
            elif choice == "5":
                self.test_individual_motor_continuous("Motor de Escoba", 
                    self.BROOM_A, self.BROOM_B, self.BROOM_PWM, speed=0.8)
            elif choice == "6":
                self.test_individual_servo_continuous("Servo Compuerta Izquierda", self.SERVO_LEFT)
            elif choice == "7":
                self.test_individual_servo_continuous("Servo Compuerta Derecha", self.SERVO_RIGHT)
            elif choice == "8":
                self.test_individual_servo_continuous("Servo Piso", self.SERVO_FLOOR)
            elif choice == "0":
                print("📋 Regresando al menú principal...")
                break
            else:
                print("❌ Opción inválida. Intenta de nuevo.")
                time.sleep(1)

    def run_all_tests(self):
        """Ejecutar todas las pruebas"""
        print("\n🚀 INICIANDO PRUEBAS DE MOTORES DE TRACCIÓN")
        print("=" * 50)
        
        # Pruebas de motores de tracción
        self.test_motor("Motor Frontal Izquierdo", 
                       self.FRONT_LEFT_A1, self.FRONT_LEFT_A2, self.FRONT_LEFT_PWMA)
        
        self.test_motor("Motor Frontal Derecho", 
                       self.FRONT_RIGHT_B1, self.FRONT_RIGHT_B2, self.FRONT_RIGHT_PWMB)
        
        self.test_motor("Motor Trasero Izquierdo", 
                       self.REAR_LEFT_A1, self.REAR_LEFT_A2, self.REAR_LEFT_PWMA)
        
        self.test_motor("Motor Trasero Derecho", 
                       self.REAR_RIGHT_B1, self.REAR_RIGHT_B2, self.REAR_RIGHT_PWMB)
        
        print("\n🧹 INICIANDO PRUEBA DE MOTOR DE ESCOBA")
        print("=" * 50)
        
        # Prueba del motor de escoba
        self.test_motor("Motor de Escoba", 
                       self.BROOM_A, self.BROOM_B, self.BROOM_PWM, speed=0.8)
        
        print("\n🚪 INICIANDO PRUEBAS DE SERVOMOTORES")
        print("=" * 50)
        
        # Pruebas de servos
        self.test_servo("Servo Compuerta Izquierda", self.SERVO_LEFT)
        self.test_servo("Servo Compuerta Derecha", self.SERVO_RIGHT)
        self.test_servo("Servo Piso", self.SERVO_FLOOR)
        
        print("\n✅ TODAS LAS PRUEBAS COMPLETADAS")

    def test_traction_motors_only(self):
        """Ejecutar solo las pruebas de motores de tracción"""
        print("\n🚀 INICIANDO PRUEBAS DE MOTORES DE TRACCIÓN")
        print("=" * 50)
        
        # Pruebas de motores de tracción
        self.test_motor("Motor Frontal Izquierdo", 
                       self.FRONT_LEFT_A1, self.FRONT_LEFT_A2, self.FRONT_LEFT_PWMA)
        
        self.test_motor("Motor Frontal Derecho", 
                       self.FRONT_RIGHT_B1, self.FRONT_RIGHT_B2, self.FRONT_RIGHT_PWMB)
        
        self.test_motor("Motor Trasero Izquierdo", 
                       self.REAR_LEFT_A1, self.REAR_LEFT_A2, self.REAR_LEFT_PWMA)
        
        self.test_motor("Motor Trasero Derecho", 
                       self.REAR_RIGHT_B1, self.REAR_RIGHT_B2, self.REAR_RIGHT_PWMB)
        
        print("\n✅ PRUEBAS DE MOTORES DE TRACCIÓN COMPLETADAS")

    def test_servos_only(self):
        """Ejecutar solo las pruebas de servomotores"""
        print("\n🚪 INICIANDO PRUEBAS DE SERVOMOTORES")
        print("=" * 50)
        
        # Pruebas de servos
        self.test_servo("Servo Compuerta Izquierda", self.SERVO_LEFT)
        self.test_servo("Servo Compuerta Derecha", self.SERVO_RIGHT)
        self.test_servo("Servo Piso", self.SERVO_FLOOR)
        
        print("\n✅ PRUEBAS DE SERVOMOTORES COMPLETADAS")

    def test_broom_motor_only(self):
        """Ejecutar solo la prueba del motor de escoba"""
        print("\n🧹 INICIANDO PRUEBA DE MOTOR DE ESCOBA")
        print("=" * 50)
        
        # Prueba del motor de escoba
        self.test_motor("Motor de Escoba", 
                       self.BROOM_A, self.BROOM_B, self.BROOM_PWM, speed=0.8)
        
        print("\n✅ PRUEBA DE MOTOR DE ESCOBA COMPLETADA")

    def test_all_motors_forward(self):
        """Prueba todos los motores de tracción hacia adelante simultáneamente"""
        print("\n🔄 PRUEBA CONJUNTA - Todos los motores adelante por 3 segundos...")
        
        # Configurar todos los motores para adelante
        self.digital_pins[self.FRONT_LEFT_A1].on()
        self.digital_pins[self.FRONT_LEFT_A2].off()
        self.digital_pins[self.FRONT_RIGHT_B1].on()
        self.digital_pins[self.FRONT_RIGHT_B2].off()
        self.digital_pins[self.REAR_LEFT_A1].on()
        self.digital_pins[self.REAR_LEFT_A2].off()
        self.digital_pins[self.REAR_RIGHT_B1].on()
        self.digital_pins[self.REAR_RIGHT_B2].off()
        
        # Aplicar PWM a todos
        self.pwm_pins[self.FRONT_LEFT_PWMA].value = 0.5
        self.pwm_pins[self.FRONT_RIGHT_PWMB].value = 0.5
        self.pwm_pins[self.REAR_LEFT_PWMA].value = 0.5
        self.pwm_pins[self.REAR_RIGHT_PWMB].value = 0.5
        
        time.sleep(3)
        
        # Detener todos
        self.stop_all_motors()
        print("✅ Prueba conjunta completada")

    def all_traction_motors_control(self):
        """Control continuo de todos los motores de tracción simultáneamente"""
        print("\n🚗 CONTROL CONJUNTO DE MOTORES DE TRACCIÓN")
        print("=" * 60)
        print("Controles disponibles:")
        print("  1 = Todos adelante")
        print("  2 = Todos atrás") 
        print("  3 = Detener todos")
        print("  4 = Girar izquierda (motores izq. atrás, der. adelante)")
        print("  5 = Girar derecha (motores izq. adelante, der. atrás)")
        print("  ESC = Salir del control")
        print("=" * 60)
        print("⚠️  Los motores permanecerán activos hasta que presiones '3' o ESC")
        print("🔧 Escribe comandos y presiona ENTER")
        
        # Usar el sistema existente de input
        self.start_input_thread()
        
        try:
            while True:
                # Verificar si se presionó una tecla usando el método existente
                key = self.get_key_pressed()
                
                if key:
                    if key.upper() == 'ESC':
                        print("🏁 Saliendo del control conjunto...")
                        break
                    elif key == '1':  # Todos adelante
                        print("▶️  Todos los motores: ADELANTE")
                        self._set_all_traction_motors_forward()
                        
                    elif key == '2':  # Todos atrás
                        print("◀️  Todos los motores: ATRÁS")
                        self._set_all_traction_motors_backward()
                        
                    elif key == '3':  # Detener
                        print("⏹️  Todos los motores: DETENIDOS")
                        self._stop_all_traction_motors()
                        
                    elif key == '4':  # Girar izquierda
                        print("↪️  Girando: IZQUIERDA")
                        self._set_traction_motors_turn_left()
                        
                    elif key == '5':  # Girar derecha
                        print("↩️  Girando: DERECHA") 
                        self._set_traction_motors_turn_right()
                        
                    else:
                        print(f"❌ Comando no reconocido: {key}")
                        
                time.sleep(0.1)  # Pequeña pausa para no saturar la CPU
                
        except KeyboardInterrupt:
            print("\n🛑 Control interrumpido por el usuario")
        finally:
            self.stop_input_thread()
            # Asegurar que todos los motores se detengan
            self._stop_all_traction_motors()
            print("✅ Control conjunto finalizado")

    def _set_all_traction_motors_forward(self):
        """Configurar todos los motores de tracción para adelante"""
        # Configurar dirección adelante para todos
        self.digital_pins[self.FRONT_LEFT_A1].on()
        self.digital_pins[self.FRONT_LEFT_A2].off()
        self.digital_pins[self.FRONT_RIGHT_B1].on()
        self.digital_pins[self.FRONT_RIGHT_B2].off()
        self.digital_pins[self.REAR_LEFT_A1].on()
        self.digital_pins[self.REAR_LEFT_A2].off()
        self.digital_pins[self.REAR_RIGHT_B1].on()
        self.digital_pins[self.REAR_RIGHT_B2].off()
        
        # Aplicar PWM
        self.pwm_pins[self.FRONT_LEFT_PWMA].value = 1
        self.pwm_pins[self.FRONT_RIGHT_PWMB].value = 1
        self.pwm_pins[self.REAR_LEFT_PWMA].value = 1
        self.pwm_pins[self.REAR_RIGHT_PWMB].value = 1

    def _set_all_traction_motors_backward(self):
        """Configurar todos los motores de tracción para atrás"""
        # Configurar dirección atrás para todos
        self.digital_pins[self.FRONT_LEFT_A1].off()
        self.digital_pins[self.FRONT_LEFT_A2].on()
        self.digital_pins[self.FRONT_RIGHT_B1].off()
        self.digital_pins[self.FRONT_RIGHT_B2].on()
        self.digital_pins[self.REAR_LEFT_A1].off()
        self.digital_pins[self.REAR_LEFT_A2].on()
        self.digital_pins[self.REAR_RIGHT_B1].off()
        self.digital_pins[self.REAR_RIGHT_B2].on()
        
        # Aplicar PWM
        self.pwm_pins[self.FRONT_LEFT_PWMA].value = 1
        self.pwm_pins[self.FRONT_RIGHT_PWMB].value = 1
        self.pwm_pins[self.REAR_LEFT_PWMA].value = 1
        self.pwm_pins[self.REAR_RIGHT_PWMB].value = 1

    def _set_traction_motors_turn_left(self):
        """Configurar motores para girar a la izquierda"""
        # Motores izquierdos atrás, derechos adelante
        self.digital_pins[self.FRONT_LEFT_A1].off()
        self.digital_pins[self.FRONT_LEFT_A2].on()
        self.digital_pins[self.FRONT_RIGHT_B1].on()
        self.digital_pins[self.FRONT_RIGHT_B2].off()
        self.digital_pins[self.REAR_LEFT_A1].off()
        self.digital_pins[self.REAR_LEFT_A2].on()
        self.digital_pins[self.REAR_RIGHT_B1].on()
        self.digital_pins[self.REAR_RIGHT_B2].off()
        
        # Aplicar PWM
        self.pwm_pins[self.FRONT_LEFT_PWMA].value = 0.5
        self.pwm_pins[self.FRONT_RIGHT_PWMB].value = 0.5
        self.pwm_pins[self.REAR_LEFT_PWMA].value = 0.5
        self.pwm_pins[self.REAR_RIGHT_PWMB].value = 0.5

    def _set_traction_motors_turn_right(self):
        """Configurar motores para girar a la derecha"""
        # Motores izquierdos adelante, derechos atrás
        self.digital_pins[self.FRONT_LEFT_A1].on()
        self.digital_pins[self.FRONT_LEFT_A2].off()
        self.digital_pins[self.FRONT_RIGHT_B1].off()
        self.digital_pins[self.FRONT_RIGHT_B2].on()
        self.digital_pins[self.REAR_LEFT_A1].on()
        self.digital_pins[self.REAR_LEFT_A2].off()
        self.digital_pins[self.REAR_RIGHT_B1].off()
        self.digital_pins[self.REAR_RIGHT_B2].on()
        
        # Aplicar PWM
        self.pwm_pins[self.FRONT_LEFT_PWMA].value = 0.5
        self.pwm_pins[self.FRONT_RIGHT_PWMB].value = 0.5
        self.pwm_pins[self.REAR_LEFT_PWMA].value = 0.5
        self.pwm_pins[self.REAR_RIGHT_PWMB].value = 0.5

    def _stop_all_traction_motors(self):
        """Detener solo los motores de tracción"""
        # Detener PWM de motores de tracción
        self.pwm_pins[self.FRONT_LEFT_PWMA].value = 0
        self.pwm_pins[self.FRONT_RIGHT_PWMB].value = 0
        self.pwm_pins[self.REAR_LEFT_PWMA].value = 0
        self.pwm_pins[self.REAR_RIGHT_PWMB].value = 0
        
        # Poner pines de control en LOW
        traction_control_pins = [
            self.FRONT_LEFT_A1, self.FRONT_LEFT_A2,
            self.FRONT_RIGHT_B1, self.FRONT_RIGHT_B2,
            self.REAR_LEFT_A1, self.REAR_LEFT_A2,
            self.REAR_RIGHT_B1, self.REAR_RIGHT_B2
        ]
        
        for pin in traction_control_pins:
            self.digital_pins[pin].off()

    def stop_all_motors(self):
        """Detener todos los motores"""
        # Detener PWM de motores
        for pin, pwm_obj in self.pwm_pins.items():
            pwm_obj.value = 0
        
        # Poner todos los pines de control en LOW
        control_pins = [
            self.FRONT_LEFT_A1, self.FRONT_LEFT_A2,
            self.FRONT_RIGHT_B1, self.FRONT_RIGHT_B2,
            self.REAR_LEFT_A1, self.REAR_LEFT_A2,
            self.REAR_RIGHT_B1, self.REAR_RIGHT_B2,
            self.BROOM_A, self.BROOM_B
        ]
        
        for pin in control_pins:
            self.digital_pins[pin].off()

    def emergency_stop(self):
        """Detener todos los dispositivos inmediatamente en caso de emergencia"""
        print("\n🚨 PARADA DE EMERGENCIA ACTIVADA")
        
        try:
            # Detener motores inmediatamente
            self.stop_all_motors()
            
            # Detener servos inmediatamente
            for pin, servo_obj in self.servo_objects.items():
                servo_obj.value = None
                
        except Exception as e:
            print(f"⚠️ Error en parada de emergencia: {e}")

    def cleanup(self):
        """Limpiar recursos GPIO"""
        print("\n🧹 Limpiando recursos GPIO...")
        
        try:
            # Detener todos los motores
            self.stop_all_motors()
            
            # Detener servos
            for pin, servo_obj in self.servo_objects.items():
                servo_obj.value = None
                servo_obj.close()
            
            # Cerrar objetos PWM
            for pin, pwm_obj in self.pwm_pins.items():
                pwm_obj.close()
            
            # Cerrar objetos digitales
            for pin, digital_obj in self.digital_pins.items():
                digital_obj.close()
            
            print("✅ GPIO limpiado correctamente")
            
        except Exception as e:
            print(f"⚠️ Error limpiando GPIO: {e}")

def signal_handler(sig, frame):
    """Manejador de señales para limpieza segura"""
    print("\n🛑 Señal de interrupción recibida")
    sys.exit(0)

def main():
    """Función principal"""
    print("🤖 PROBADOR DE PINES DE MOTORES - ROBOBEACH (gpiozero)")
    print("=" * 60)
    print()
    
    # Configurar manejador de señales
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    tester = None
    try:
        tester = MotorPinTester()
        
        print("\n🔧 MENÚ DE PRUEBAS:")
        print("1. Pruebas individuales completas (recomendado)")
        print("2. Solo motores de tracción")
        print("3. Solo servomotores")
        print("4. Solo motor de escoba")
        print("5. Prueba conjunta de motores")
        print("6. Todas las pruebas")
        print("7. Control individual continuo (ESC para salir)")
        print("8. Control conjunto de motores de tracción (ESC para salir)")
        
        choice = input("\nSelecciona una opción (1-8): ").strip()
        
        if choice == "1":
            tester.run_all_tests()
        elif choice == "2":
            tester.test_traction_motors_only()
        elif choice == "3":
            tester.test_servos_only()
        elif choice == "4":
            tester.test_broom_motor_only()
        elif choice == "5":
            tester.test_all_motors_forward()
        elif choice == "6":
            tester.run_all_tests()
            tester.test_all_motors_forward()
        elif choice == "7":
            tester.individual_motor_menu()
        elif choice == "8":
            tester.all_traction_motors_control()
        else:
            print("❌ Opción inválida")
            return
            
    except KeyboardInterrupt:
        print("\n🛑 Pruebas interrumpidas por el usuario")
        if tester:
            tester.emergency_stop()
    except Exception as e:
        print(f"\n❌ Error durante las pruebas: {e}")
        if tester:
            tester.emergency_stop()
    finally:
        if tester:
            tester.cleanup()
        print("\n🏁 Pruebas finalizadas")

if __name__ == "__main__":
    main()
