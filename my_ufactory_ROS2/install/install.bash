#!/bin/bash

# Save the current directory
current_dir=$(pwd)
cd ../..

# Update package lists
sudo apt update

# Install dependencies and build the workspace
cd ..
rosdep init
rosdep update
rosdep install --ignore-src --from-paths src -y



cd "$current_dir"

if ros2 pkg list | grep -q "xarm_description"; then
    echo "xarm packages alredy installed"
else
    echo "cloning xarm packages"
    git clone https://github.com/xArm-Developer/xarm_ros2.git ../../xarm_ros -b $ROS_DISTRO --recursive 
    #git clone https://github.com/ros-planning/moveit_task_constructor.git ../../moveit_task_constructor -b $ROS_DISTRO

    cd ../../xarm_ros
    git pull
    git submodule sync
    git submodule update --init --remote
fi

cd "$current_dir"
if ros2 pkg list | grep -q "pymoveit2"; then
    echo "pymoveit2 packages alredy installed"
else
    echo "cloning xarm pymoveit2"
    git clone https://github.com/AvansMechatronica/pymoveit2.git ../../pymoveit2 
fi

cd "$current_dir"
if ros2 pkg list | grep -q "my_moveit_python"; then
    echo "my_moveit_python packages alredy installed"
else
    echo "cloning my_moveit_python"
    git clone https://github.com/AvansMechatronica/my_moveit_python.git ../../my_moveit_python 
fi

cd "$current_dir"
cd ../../..
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

# Set QT_QPA_PLATFORM to xcb
if ! env | grep -q "QT_QPA_PLATFORM=xcb"; then
    echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc
fi
