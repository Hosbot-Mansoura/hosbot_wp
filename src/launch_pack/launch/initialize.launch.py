import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    ld = LaunchDescription()
    urdf_file_name = "robot.urdf.xacro"
    package_name = "robot_urdf"
    hardware_package_path = get_package_share_directory('hardware_pkg')
    robot_description_file_path = os.path.join(get_package_share_directory(package_name),"urdf",urdf_file_name)
    hardware_config_files = [os.path.join(hardware_package_path,'config','params.yaml')]

    doc = xacro.process_file(robot_description_file_path)

    state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        parameters=[
            {'use_sim_time':False , 'robot_description':doc.toxml()}
        ],
        output = "screen"
    )

    motors_node = Node(
        package='hardware_pkg',
        executable='motors_node',
        name = "motors_node",
        parameters=hardware_config_files,
        output = 'screen'
    )

    encoder_node = Node(
        package='hardware_pkg',
        executable='encoder_node',
        name = "encoder_node",
        parameters=hardware_config_files,
        output = 'screen'
    )

    imu_node = Node(
        package='hardware_pkg',
        executable='imu_node',
        name = "imu_node",
        parameters=hardware_config_files,
        output = 'screen'
    )


    ldlidar_launch = IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource([
          get_package_share_directory('ldlidar_ros2'),
          '/launch/ld06.launch.py'
      ])
    )
    
    ld.add_action(state_publisher_node)
    ld.add_action(motors_node)
    ld.add_action(encoder_node)
    ld.add_action(imu_node)
    ld.add_action(ldlidar_launch)
    return ld


