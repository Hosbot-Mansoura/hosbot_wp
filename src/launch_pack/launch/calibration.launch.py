import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch = LaunchDescription()
    hardware_package_path = get_package_share_directory('hardware_pkg')
    parameters_file = [os.path.join(hardware_package_path,'config','params.yaml')]

    calibration_node = Node(
        package="hardware_pkg",
        executable="calibrate_motors_node",
        name="calibrate_motors_node",
        output="screen"
    )

    encoder_node = Node(
        package= "hardware_pkg",
        executable="encoder_node",
        name="encoder_node",
        parameters= parameters_file,
        output="screen"
    )

    launch.add_action(encoder_node)
    launch.add_action(calibration_node)
    return launch