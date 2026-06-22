#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    uf_bringup_share = get_package_share_directory(
        "my_uf_bringup"
    )

    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                uf_bringup_share,
                "launch",
                "real_robot.launch.py",
            )
        )
    )

    manipulator_node = Node(
        package="manipulatie",
        executable="manipulator",
        name="manipulator",
        output="screen",
    )

    delayed_manipulator = TimerAction(
        period=15.0,
        actions=[manipulator_node],
    )

    return LaunchDescription([
        real_robot_launch,
        delayed_manipulator,
    ])