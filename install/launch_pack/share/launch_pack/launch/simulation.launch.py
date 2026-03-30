import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
        urdf_file_name = "robot.urdf.xacro"
    package_name = "robot_urdf"

    robot_description_file_path = os.path.join(get_package_share_directory(package_name),"urdf",urdf_file_name)
    doc = xacro.process_file(robot_description_file_path)

    # urdf_file_content = open(robot_description_file_path).read()
    state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        parameters=[
            {'use_sim_time':True , 'robot_description':doc.toxml()}
        ],
        output = "screen"
    )

    return LaunchDescription([state_publisher_node])
    pass