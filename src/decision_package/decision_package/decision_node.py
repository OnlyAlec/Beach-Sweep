"""
Módulo Logic para sistema de recolección de basura con ROS

Responsable de:
1. Procesar datos de Sensor (distancia) y AIVision (detecciones YOLO)
2. Tomar decisiones de navegación y manipulación
3. Enviar comandos al módulo Motors
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
from vision_ia_msgs.msg import Detections, Detection
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, Duration

DISTANCIA_SEGURA = 0.4  # Distancia mínima para detección de obstáculos (metros)
CAPACIDAD_MAX = 3       # Máximo de latas que puede transportar el robot

# Estados posibles
BUSCAR_LATA       = "buscar_lata"       # Buscar latas disponibles
IR_A_LATA         = "ir_a_lata"         # Navegar hacia una lata detectada
BUSCAR_CONTENEDOR = "buscar_contenedor" # Buscar contenedor rojo
IR_A_CONTENEDOR   = "ir_a_contenedor"   # Navegar hacia el contenedor
EVITAR            = "evitar"            # Realizar maniobra de evasión

class LogicNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        
        # Configuración de QoS para mejor compatibilidad
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            deadline=Duration(seconds=0)
        )
        
        # Suscriptores
        self.detecciones_sub = self.create_subscription(
            Detections,
            'detections',
            self.callback_detecciones,
            qos)
            
        self.distancia_sub = self.create_subscription(
            Range,
            'distance',
            self.callback_distancia,
            qos)
            
        # Publicador de comandos
        self.comandos_pub = self.create_publisher(String, 'control', qos)
        
        # Variables de estado
        self.detecciones_actuales = []
        self.distancia_actual = float('inf')
        self.estado_actual = BUSCAR_LATA
        self.estado_anterior = None
        self.lata_recogida = False
        self.contador_latas = 0
        
        # Timer para el loop de lógica
        self.timer = self.create_timer(0.1, self.logic_loop)  # 10 Hz
        
    def callback_detecciones(self, msg):
        """Callback para procesar detecciones de YOLO desde AIVision.
    
        Args:
            msg (DetectionArray): Mensaje ROS con lista de detecciones
        """
        self.detecciones_actuales = msg.detections
        self.get_logger().debug(f'Detecciones recibidas: {len(self.detecciones_actuales)}')

    def callback_distancia(self, msg):
        """Callback para procesar datos del sensor de distancia.
    
        Args:
            msg (Range): Mensaje ROS con distancia medida
        """
        self.distancia_actual = msg.range
        self.get_logger().debug(f'Distancia actualizada: {self.distancia_actual:.2f} m')

    def filtrar(self, clase_objetivo):
        """Filtra objetos por clase de interés.
    
        Args:
            lista (list): Lista de detecciones
            clase_objetivo (str): Clase a filtrar (ej: 'black_can')
        
        Returns:
            list: Subconjunto de detecciones que coinciden con la clase
        """
        return [obj for obj in self.detecciones_actuales if obj.class_name == clase_objetivo]

    def es_frontal(self, obj):
        """Determina si un objeto está en la zona frontal del robot.
    
        Criterios:
        - Centro X entre 45% y 55% del ancho de imagen
        - Centro Y arriba del 60% de la altura
        
        Args:
            obj (Detection): Objeto detectado
        
        Returns:
            bool: True si está en zona frontal
        """
        centro_x = obj.x + obj.width / 2
        centro_y = obj.y + obj.height / 2
        return 0.45 < centro_x < 0.55 and centro_y > 0.6

    def obtener_frontal(self, lista):
        """Selecciona el objeto frontal más grande.
    
        Args:
            lista (list): Lista de detecciones
        
        Returns:
            Detection or None: Objeto frontal más grande o None
        """
        frontales = [obj for obj in lista if self.es_frontal(obj)]
        return max(frontales, key=lambda o: o.width*o.height) if frontales else None

    def seleccionar_objeto_prioritario(self, lista):
        """Selecciona el objeto más grande por área en imagen.
        Args:
            lista (list): Lista de detecciones
        
        Returns:
            Detection or None: Objeto prioritario o None
        """
        return max(lista, key=lambda o: o.width * o.height) if lista else None

    def centrar_con_objetivo(self, obj, img_width=640):
        """Calcula comando de movimiento para centrar el objeto en visión.
    
        Args:
            obj (Detection): Objeto a centrar
            img_width (int): Ancho de imagen en píxeles
        
        Returns:
            str: Comando de movimiento ('FORWARD' o 'LEFT')
        """
        centro_x_norm = obj.x + obj.width / 2
        centro_x_px = centro_x_norm * img_width
        
        CENTRO_IMAGEN = img_width / 2
        OFFSET_MAX = 200
        ANGULO_MAX = 45
        
        offset = centro_x_px - CENTRO_IMAGEN
        angulo = (offset / OFFSET_MAX) * ANGULO_MAX
        angulo = max(min(angulo, ANGULO_MAX), -ANGULO_MAX)
        
        if abs(angulo) > 5:
            return "LEFT"
        else:
            return "FORWARD"

    def publicar_orden(self, comando):
        """Publica un comando en el topic de Motors.
        
        Args:
            comando (str): Comando a ejecutar
        """
        msg = String()
        msg.data = comando
        self.comandos_pub.publish(msg)
        self.get_logger().info(f'Enviando comando: {comando}')

    def logic_loop(self):
        """Máquina de estados principal ejecutada en loop."""

        latas        = self.filtrar("black_can")
        contenedores = self.filtrar("red_container")
        
        lata_frontal       = self.obtener_frontal(latas)
        contenedor_frontal = self.obtener_frontal(contenedores)

        if self.estado_actual == BUSCAR_LATA:
            """Estado 1: Búsqueda inicial de latas"""
            if self.contador_latas >= CAPACIDAD_MAX:
                self.estado_actual = BUSCAR_CONTENEDOR
            elif not latas:
                self.publicar_orden("SPIRAL")
            else:
                objetivo_actual = self.seleccionar_objeto_prioritario(latas)
                orden = self.centrar_con_objetivo(objetivo_actual)
                self.publicar_orden(orden)
                if "FORWARD" in orden:
                    self.estado_actual = IR_A_LATA

        elif self.estado_actual == IR_A_LATA:
            """Estado 2: Navegación hacia lata"""
            if self.distancia_actual < DISTANCIA_SEGURA:
                if self.contador_latas >= CAPACIDAD_MAX:
                    self.estado_actual = BUSCAR_CONTENEDOR
                elif lata_frontal:
                    self.publicar_orden("PICKUP")
                    self.contador_latas += 1
                    self.estado_actual = BUSCAR_LATA
                else:
                    self.estado_anterior = self.estado_actual
                    self.estado_actual = EVITAR
            else:
                objetivo_actual = self.seleccionar_objeto_prioritario(latas)
                if objetivo_actual:
                    orden = self.centrar_con_objetivo(objetivo_actual)
                    self.publicar_orden(orden)
                else:
                    self.estado_actual = BUSCAR_LATA

        elif self.estado_actual == BUSCAR_CONTENEDOR:
            """Estado 3: Búsqueda de contenedor"""
            if not contenedores:
                self.publicar_orden("SPIRAL")
            else:
                objetivo = self.seleccionar_objeto_prioritario(contenedores)
                orden = self.centrar_con_objetivo(objetivo)
                self.publicar_orden(orden)
                if "FORWARD" in orden:
                    self.estado_actual = IR_A_CONTENEDOR

        elif self.estado_actual == IR_A_CONTENEDOR:
            """Estado 4: Navegación hacia contenedor"""
            if self.distancia_actual < DISTANCIA_SEGURA and contenedor_frontal:
                self.publicar_orden("DEPOSIT")
                self.contador_latas = 0
                self.estado_actual = BUSCAR_LATA
            elif self.distancia_actual < DISTANCIA_SEGURA:
                self.estado_anterior = self.estado_actual
                self.estado_actual = EVITAR
            else:
                objetivo_actual = self.seleccionar_objeto_prioritario(contenedores)
                if objetivo_actual:
                    orden = self.centrar_con_objetivo(objetivo_actual)
                    self.publicar_orden(orden)
                else:
                    self.estado_actual = BUSCAR_CONTENEDOR

        elif self.estado_actual == EVITAR:
            """Estado 5: Manejo de obstáculos"""
            if self.distancia_actual < DISTANCIA_SEGURA:
                self.publicar_orden("LEFT")
            else:
                self.estado_actual = self.estado_anterior

def main(args=None):
    rclpy.init(args=args)
    node = LogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo interrumpido por el usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()