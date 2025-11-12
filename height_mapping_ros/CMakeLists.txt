cmake_minimum_required(VERSION 3.16)
project(height_mapping)

# --- C++ ---
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# --- Dependencias ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp QUIET)  # opcional; enlazamos si existe
find_package(height_mapping_core REQUIRED)  # si es un paquete ament separado

# --- Interfaces (srv) ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# --- Includes ---
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# --- Ejecutables ---
# Nota: mantiene las rutas de tu árbol original. Ajusta si moviste archivos.
add_executable(height_mapping_node
  src/ros/height_mapping_node.cpp
  src/core/HeightMapper.cpp
)

add_executable(global_mapping_node
  src/ros/global_mapping_node.cpp
  src/core/HeightMapper.cpp
  src/core/GlobalMapper.cpp
)

add_executable(sensor_processor_node
  src/ros/sensor_processor_node.cpp
)

# --- Dependencias de los targets ---
ament_target_dependencies(height_mapping_node
  rclcpp tf2 tf2_ros tf2_eigen 
  tf2_geometry_msgs geometry_msgs sensor_msgs
  grid_map_core grid_map_msgs grid_map_ros
  PCL pcl_conversions
  height_mapping_core
)

ament_target_dependencies(global_mapping_node
  rclcpp tf2 tf2_ros tf2_eigen 
  tf2_geometry_msgs geometry_msgs sensor_msgs visualization_msgs
  grid_map_core grid_map_msgs grid_map_ros std_srvs
  PCL pcl_conversions
  rosbag2_cpp ament_index_cpp
  height_mapping_core
)

ament_target_dependencies(sensor_processor_node
  rclcpp geometry_msgs tf2 tf2_ros tf2_eigen 
  tf2_geometry_msgs sensor_msgs
  message_filters
  PCL pcl_conversions
  height_mapping_core
)

# yaml-cpp (si está disponible)
if(TARGET YAML::yaml-cpp)
  target_link_libraries(global_mapping_node YAML::yaml-cpp)
endif()

# Tipos de soporte de rosidl para enlazar (por si el nodo usa el srv/msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(global_mapping_node ${cpp_typesupport_target})
# (height_mapping_node podría no necesitar el srv; enlázalo si lo incluyes)
target_link_libraries(height_mapping_node ${cpp_typesupport_target})

# --- Instalación ---
install(TARGETS
  height_mapping_node
  global_mapping_node
  sensor_processor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)

# --- Paquete ---
ament_package()
