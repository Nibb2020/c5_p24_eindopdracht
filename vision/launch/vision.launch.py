# =========================================================
# Imports
# =========================================================

from launch import LaunchDescription  # Basisobject voor ROS2 launchbestanden
from launch_ros.actions import Node  # Actie om een ROS2 node te starten

# =========================================================
# Launch Description
# =========================================================

def generate_launch_description():

    vision_node = Node(  # Definieer de vision node
        package="vision",  # ROS2 package naam
        executable="vision_node",  # Executable uit setup.py
        name="vision_node",  # Node naam in ROS2 graph
        output="screen",  # Print logs naar terminal
        emulate_tty=True,  # Zorgt voor nette terminaloutput
        parameters=[]  # Reserveplek voor latere ROS parameters
    )

    return LaunchDescription([  # Geef launchconfiguratie terug
        vision_node  # Start vision_node
    ])