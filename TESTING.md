# 🤖 Guía de Testing - Sistema RoboBeach

## 📋 Estructura del Proyecto Unificado

```
ULSA_RoboBeach/
├── src/
│   ├── robot_interfaces/          # Mensajes personalizados (CMake)
│   │   ├── msg/
│   │   │   ├── Detection.msg      # Detección individual
│   │   │   └── Detections.msg     # Array de detecciones
│   │   └── CMakeLists.txt
│   ├── vision_ia_core/           # Visión y coordinación (Python)
│   │   ├── vision_ia_core/
│   │   │   ├── camera_node.py    # Captura de video
│   │   │   └── detect_node.py    # Detección YOLO
│   │   └── launch/
│   │       ├── vision.launch.py          # Solo visión
│   │       └── robot_system.launch.py   # Sistema completo
│   ├── sensor_package/           # Sensor LiDAR (Python)
│   │   └── launch/sensor.launch.py
│   ├── motor_package/            # Control motores (Python)
│   │   └── launch/motor.launch.py
│   └── decision_package/         # Lógica decisiones (Python)
│       └── launch/decision.launch.py
```

## 🔧 Configuración del Entorno

### **1. Preparación inicial**

```bash
# Cargar entorno ROS2
source /opt/ros/humble/setup.zsh

# Navegar al workspace
cd /home/alec/Documents/ULSA/ULSA_RoboBeach

# Compilar proyecto completo
colcon build

# Cargar workspace compilado
source install/setup.zsh
```

### **2. Verificar dependencias hardware**

```bash
# Verificar puerto LiDAR TF-Luna
ls -la /dev/ttyUSB*

# Verificar permisos GPIO (Raspberry Pi)
ls -la /dev/gpiomem

# Verificar archivos de datos
ls -la src/vision_ia_core/data/
```

## 🧪 Testing Individual por Paquete

### **A. Testing Robot Interfaces (Mensajes)**

```bash
# Verificar generación de mensajes
ros2 interface list | grep robot_interfaces

# Verificar estructura de mensajes
ros2 interface show robot_interfaces/msg/Detection
ros2 interface show robot_interfaces/msg/Detections

# Salida esperada:
# string class_name
# float32 x, y, width, height, confidence
# int32 class_id
```

### **B. Testing Sensor Package**

```bash
# 1. Lanzar solo el sensor
ros2 launch sensor_package sensor.launch.py

# 2. En otra terminal, verificar publicación
ros2 topic echo robot/sensors/distance
ros2 topic hz robot/sensors/distance    # ~10Hz esperado

# 3. Verificar tipos de mensaje
ros2 topic info robot/sensors/distance
# Tipo esperado: sensor_msgs/msg/Range

# 4. Testing manual con objeto
# Acercar/alejar objeto del sensor y verificar cambios
```

### **C. Testing Motor Package**

```bash
# 1. Lanzar solo motores
ros2 launch motor_package motor.launch.py

# 2. En otra terminal, enviar comandos de prueba
ros2 topic pub robot/motor/commands std_msgs/String "data: 'FORWARD'" --once
ros2 topic pub robot/motor/commands std_msgs/String "data: 'LEFT'" --once
ros2 topic pub robot/motor/commands std_msgs/String "data: 'RIGHT'" --once
ros2 topic pub robot/motor/commands std_msgs/String "data: 'BACKWARD'" --once
ros2 topic pub robot/motor/commands std_msgs/String "data: 'STOP'" --once

# 3. Comandos especiales
ros2 topic pub robot/motor/commands std_msgs/String "data: 'PICKUP'" --once
ros2 topic pub robot/motor/commands std_msgs/String "data: 'DEPOSIT'" --once
ros2 topic pub robot/motor/commands std_msgs/String "data: 'SPIRAL'" --once

# Resultado esperado: Logs del nodo indicando recepción de comandos
```

### **D. Testing Vision Package**

```bash
# 1. Lanzar solo visión
ros2 launch vision_ia_core vision.launch.py

# 2. Verificar tópicos de video
ros2 topic echo robot/vision/frames --max-count=1  # Un frame
ros2 topic hz robot/vision/frames                  # ~30Hz esperado

# 3. Verificar detecciones YOLO
ros2 topic echo robot/vision/detections
ros2 topic hz robot/vision/detections              # Variable según detecciones

# 4. Verificar estructura de detecciones
# Debe mostrar arrays de objetos detectados con:
# - class_name: "can", "bottle", "container"
# - coordenadas x, y, width, height
# - confidence > 0.5
```

### **E. Testing Decision Package**

```bash
# 1. Lanzar solo decisiones
ros2 launch decision_package decision.launch.py

# 2. Verificar suscripciones
ros2 node info /decision_node

# 3. Monitorear comandos generados
ros2 topic echo robot/motor/commands

# 4. Simular datos de entrada
# Terminal 1: Simular sensor
ros2 topic pub robot/sensors/distance sensor_msgs/Range '{header: {frame_id: "sensor"}, radiation_type: 1, field_of_view: 0.1, min_range: 0.02, max_range: 8.0, range: 2.0}' --rate 10

# Terminal 2: Simular detecciones (vacías para testing)
ros2 topic pub robot/vision/detections robot_interfaces/Detections '{header: {frame_id: "camera"}, detections: []}' --rate 5

# Resultado esperado: Comandos "SPIRAL" (búsqueda)
```

## 🎯 Testing Sistema Completo

### **1. Lanzamiento del sistema**

```bash
# Lanzar todo el sistema
ros2 launch vision_ia_core robot_system.launch.py

# Verificar que todos los nodos estén activos
ros2 node list
# Esperado:
# /camera_node
# /detect_node
# /sensor_node
# /decision_node
# /motor_controller
```

### **2. Monitoreo de comunicación**

```bash
# Verificar comunicación entre nodos
rqt_graph

# Listar todos los tópicos activos
ros2 topic list
# Esperado:
# /robot/vision/frames
# /robot/vision/detections
# /robot/sensors/distance
# /robot/motor/commands

# Monitorear flujo de datos
ros2 topic hz /robot/vision/frames      # ~30Hz
ros2 topic hz /robot/vision/detections  # Variable
ros2 topic hz /robot/sensors/distance   # ~10Hz
ros2 topic hz /robot/motor/commands     # Variable
```

### **3. Testing de integración**

```bash
# Monitorear el flujo completo de decisiones
ros2 topic echo /robot/motor/commands

# En paralelo, observar entradas:
ros2 topic echo /robot/vision/detections --max-count=1
ros2 topic echo /robot/sensors/distance --max-count=1

# Secuencia esperada:
# 1. Detecciones de objetos → Comandos de navegación
# 2. Sensor de distancia → Comandos de evasión/recolección
# 3. Estados: SPIRAL → FORWARD → LEFT/RIGHT → PICKUP/DEPOSIT
```

## 📊 Verificación de Rendimiento

### **Frecuencias esperadas:**

- **Cámara**: ~30 FPS
- **Detecciones**: Variable (5-30 Hz según objetos)
- **Sensor**: ~10 Hz
- **Comandos**: Variable (1-10 Hz según estado)

### **Latencias aceptables:**

- **Visión → Decisión**: < 100ms
- **Sensor → Decisión**: < 50ms
- **Decisión → Motor**: < 10ms

## 🚨 Solución de Problemas Comunes

### **Error: "No module named 'robot_interfaces'"**

```bash
# Recompilar interfaces primero
colcon build --packages-select robot_interfaces
source install/setup.zsh
colcon build
```

### **Error: Video no encontrado**

```bash
# Verificar archivo de video
ls -la /home/alec/Documents/ULSA/ULSA_RoboBeach/src/vision_ia_core/data/
# Debe contener: yolo11n.pt y video_latas.mp4
```

### **Error: Puerto serial ocupado**

```bash
# Verificar procesos usando puerto
sudo lsof /dev/ttyUSB0
# Terminar procesos conflictivos si es necesario
```

### **Error: Sin detecciones YOLO**

```bash
# Verificar modelo YOLO
ls -la src/vision_ia_core/data/yolo11n.pt
# Si no existe, descargar modelo compatible
```

## ✅ Criterios de Éxito

### **✅ Compilación exitosa:**

- Sin errores en `colcon build`
- Todos los paquetes compilados

### **✅ Nodos funcionales:**

- Todos aparecen en `ros2 node list`
- Sin errores en logs

### **✅ Comunicación activa:**

- Datos fluyen en todos los tópicos
- Frecuencias dentro de rangos esperados

### **✅ Lógica operativa:**

- Detecciones generan comandos apropiados
- Sensor activa evasión de obstáculos
- Estados de decisión cambian correctamente

### **✅ Integración hardware:**

- Sensor LiDAR lee distancias
- GPIO responde a comandos (en Raspberry Pi)
- Video procesa correctamente
