# Estructura de mensaje para múltiples detecciones de objetos
# Contiene un array de detecciones individuales con timestamp
#
# Campos:
#   - header: Información de timestamp y frame_id
#   - detections: Array de detecciones individuales

std_msgs/Header header
vision_ia_core/Detection[] detections
