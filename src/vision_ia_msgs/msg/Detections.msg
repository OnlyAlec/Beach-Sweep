# =========================================================================
# MESSAGE STUCTURE FOR DETECTIONS
# =========================================================================
# Se define la estructura de un mensaje de detecciones
# Contiene información sobre:
#     - La cabecera del mensaje
#     - Un array de detecciones

std_msgs/Header header
vision_ia_msgs/Detection[] detections