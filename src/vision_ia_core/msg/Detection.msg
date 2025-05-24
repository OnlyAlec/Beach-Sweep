# Estructura de mensaje para una detección individual de objetos
# Usado por YOLO para reportar objetos detectados en la imagen
# 
# Campos:
#   - class_name: Nombre de la clase detectada (ej: "can", "bottle", "container")
#   - x, y: Coordenadas del centro del bounding box (píxeles)
#   - width, height: Dimensiones del bounding box (píxeles)
#   - confidence: Nivel de confianza de la detección (0.0 - 1.0)
#   - class_id: ID numérico de la clase

string class_name
float32 x
float32 y
float32 width
float32 height
float32 confidence
int32 class_id