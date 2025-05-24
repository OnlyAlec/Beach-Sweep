cmake_minimum_required(VERSION 3.8)
project(vision_ia_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encuentra dependencias necesarias para este paquete
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

# Genera los archivos de mensajes
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/Detections.msg"
  DEPENDENCIES std_msgs
)

# Instala los módulos Python
ament_python_install_package(${PROJECT_NAME})

# Instala scripts ejecutables
install(PROGRAMS
  vision_ia_core/camera_node.py
  vision_ia_core/detect_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instala archivos de datos
install(DIRECTORY
  data/
  DESTINATION share/${PROJECT_NAME}/data
)

# Instala archivos de lanzamiento
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
