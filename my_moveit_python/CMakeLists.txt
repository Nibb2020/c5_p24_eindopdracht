cmake_minimum_required(VERSION 3.5)
project(my_moveit_python)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(my_moveit_python)

# Setup the project
ament_package()
