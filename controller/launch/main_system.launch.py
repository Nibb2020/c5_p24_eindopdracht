#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    supervisor_node = Node(
        package="controller",
        executable="system_supervisor",
        name="system_supervisor",
        output="screen",
    )

    state_machine_node = Node(
        package="controller",
        executable="state_machine",
        name="robot_controller",
        output="screen",
    )

    return LaunchDescription([
        supervisor_node,
        state_machine_node,
    ])