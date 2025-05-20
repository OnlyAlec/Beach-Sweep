# =========================================================================
# MESSAGE STUCTURE FOR A DETECTION
# =========================================================================
# Se define la estructura de un mensaje de detección
# Contiene información sobre: 
#     - La clase del objeto detectado (nombre)
#     - Las coordinadas de la Bounding Box del objeto detectado
#     - La confianza de la detección
#     - El ID de la clase

string class_name
float32 x
float32 y
float32 width
float32 height
float32 confidence
int32 class_id