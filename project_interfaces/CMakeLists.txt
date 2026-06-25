cmake_minimum_required(VERSION 3.8)
project(project_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Dependencies
find_package(ament_cmake REQUIRED)  # Basis voor ament_cmake package
find_package(rosidl_default_generators REQUIRED)  # Nodig voor msg/srv generatie
find_package(geometry_msgs REQUIRED)  # Nodig voor geometry_msgs/TransformStamped

set(interface_files
  "srv/VoorwerpData.srv"
  "srv/Manipulator.srv"
  "srv/GetObjectData.srv"
  "msg/ObjectData.msg"
  "msg/ObjectDataArray.msg"
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

ament_package()
