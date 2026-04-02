import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch = LaunchDescription()
    hardware_package_path = get_package_share_directory('hardware_pkg')

    motors_node = Node(
        package="hardware_pkg",
        executable="motors_node",
        name="motors_node",
        parameters= [os.path.join(hardware_package_path,'config','params.yaml')],
        output="screen"
    )

    keyboard_controller_node = Node(
        package="hardware_pkg",
        executable="keyboard_controller_node",
        name="keyboard_controller_node",
        output = "screen"
    )

    launch.add_action(motors_node)
    launch.add_action(keyboard_controller_node)
    return launch