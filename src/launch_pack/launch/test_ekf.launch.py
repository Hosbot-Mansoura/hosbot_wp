from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ekf_config = os.path.join(
        os.getenv('HOME'),
        'hardware_pkg/src/hardware_pkg/config/ekf_params.yaml'
    )

    return LaunchDescription([

        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),

        # Static TF (base_link -> imu_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'imu_link']
        ),

    ])