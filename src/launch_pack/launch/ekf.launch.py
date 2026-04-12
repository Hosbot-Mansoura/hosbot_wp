from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

# must initialize robot before launching ekf to ensure that all the required topics are being published and subscribed to by the ekf node
def generate_launch_description():
    ld = LaunchDescription()
    hardware_package_path = get_package_share_directory('hardware_pkg')
    ekf_config_files = [os.path.join(hardware_package_path,'config','ekf_params.yaml')]

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=ekf_config_files
    )

    ld.add_action(ekf_node)
    return ld