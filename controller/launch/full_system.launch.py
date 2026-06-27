#!/usr/bin/env python3

# =========================================================
# Imports
# =========================================================

from launch import LaunchDescription  # Basisobject voor ROS2 launchbestanden
from launch.actions import IncludeLaunchDescription  # Laat andere launch files starten
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Python launch source
from ament_index_python.packages import get_package_share_directory  # Zoekt package share directories

from pathlib import Path  # Voor veilige bestandspaden


# =========================================================
# Launch Description
# =========================================================

def generate_launch_description():
    vision_launch_path = Path(  # Pad naar vision.launch.py
        get_package_share_directory("vision")
    ) / "launch" / "vision.launch.py"

    main_system_launch_path = Path(  # Pad naar main_system.launch.py
        get_package_share_directory("controller")
    ) / "launch" / "main_system.launch.py"

    vision_launch = IncludeLaunchDescription(  # Include vision launch
        PythonLaunchDescriptionSource(str(vision_launch_path))
    )

    main_system_launch = IncludeLaunchDescription(  # Include controller/HMI/manipulator launch
        PythonLaunchDescriptionSource(str(main_system_launch_path))
    )

    return LaunchDescription([
        vision_launch,
        main_system_launch,
    ])