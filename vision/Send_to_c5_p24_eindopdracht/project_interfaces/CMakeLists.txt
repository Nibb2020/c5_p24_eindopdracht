cmake_minimum_required(VERSION 3.8)  # Minimale CMake versie

project(project_interfaces)  # Naam van het interface package

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")  # Controleer compiler
  add_compile_options(-Wall -Wextra -Wpedantic)  # Zet compiler warnings aan
endif()

find_package(ament_cmake REQUIRED)  # Zoek ament_cmake
find_package(rosidl_default_generators REQUIRED)  # Zoek ROS interface generator
find_package(geometry_msgs REQUIRED)  # Zoek geometry_msgs voor TransformStamped

rosidl_generate_interfaces(${PROJECT_NAME}  # Genereer interfaces voor dit package
  "msg/ObjectData.msg"  # Genereer ObjectData message
  "msg/ObjectDataArray.msg"  # Genereer ObjectDataArray message
  "srv/GetObjectData.srv"  # Genereer GetObjectData service
  DEPENDENCIES geometry_msgs  # Gebruik geometry_msgs in gegenereerde interfaces
)

ament_export_dependencies(rosidl_default_runtime)  # Exporteer runtime dependency

ament_package()  # Sluit packageconfiguratie af