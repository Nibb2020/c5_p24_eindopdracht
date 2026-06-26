cmake_minimum_required(VERSION 3.8)  # Minimale CMake versie
project(project_interfaces)  # Package naam

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")  # Compilercheck
  add_compile_options(-Wall -Wextra -Wpedantic)  # Compilerwarnings aanzetten
endif()

# Dependencies
find_package(ament_cmake REQUIRED)  # Basis voor ament_cmake package
find_package(rosidl_default_generators REQUIRED)  # Nodig voor msg/srv generatie
find_package(geometry_msgs REQUIRED)  # Nodig voor geometry_msgs/TransformStamped

# Interface files
set(interface_files
  "srv/VoorwerpData.srv"  # Manipulator service
  "srv/Manipulator.srv"  # Manipulator service
  "srv/GetObjectData.srv"  # Vision service
  "msg/ObjectData.msg"  # Vision object message
  "msg/ObjectDataArray.msg"  # Vision object array message
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}  # Alle interfacebestanden
  DEPENDENCIES geometry_msgs  # Externe message dependency
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  # Zoek linting package
  set(ament_cmake_copyright_FOUND TRUE)  # Sla copyrightcheck over
  set(ament_cmake_cpplint_FOUND TRUE)  # Sla cpplint over
  ament_lint_auto_find_test_dependencies()  # Activeer lint tests
endif()

ament_export_dependencies(rosidl_default_runtime)  # Exporteer gegenereerde interface runtime
ament_package()  # Sluit package af
