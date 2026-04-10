from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

# must publish robot description first
# ros2 launch launch_pack simulation.launch.py

def generate_launch_description():
    ld = LaunchDescription()
    hardware_package_path = get_package_share_directory('hardware_pkg')
    ekf_config_files = [os.path.join(hardware_package_path,'config','ekf_params.yaml')]
    imu_config_files = [os.path.join(hardware_package_path,'config','params.yaml')]


    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=ekf_config_files
    )

    imu_node = Node(
        package='hardware_pkg',
        executable='imu_node',
        name = "imu_node",
        parameters=imu_config_files,
        output = 'screen'
    )

    encoder_node = Node(
        package='hardware_pkg',
        executable='encoder_node',
        name = "encoder_node",
        parameters=imu_config_files,
        output = 'screen'
    )

    motors_node = Node(
        package='hardware_pkg',
        executable='motors_node',
        name = "motors_node",
        parameters=imu_config_files,
        output = 'screen'
    )

    ld.add_action(imu_node)
    ld.add_action(ekf_node)
    ld.add_action(encoder_node)
    ld.add_action(motors_node)
    return ld