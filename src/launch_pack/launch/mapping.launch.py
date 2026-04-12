import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch = LaunchDescription()
    hardware_package_path = get_package_share_directory('hardware_pkg')
    map_params_file = [os.path.join(hardware_package_path,'config','map_params.yaml')]
    
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox_node",
        parameters= [
            map_params_file,
            {'use_sim_time': True}
        ],
        output="screen"
    )


    launch.add_action(slam_toolbox_node)

    return launch