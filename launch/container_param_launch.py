"""
Launch a ComposableNode with parameters and remappings.

Limitations:
 -- stdout is not set to flush after each line, so the most recent log messages may be delayed
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Set your camera namespace
camera_name = 'my_camera'

# Location of configuration directory
config_dir = os.path.join(get_package_share_directory('gscam2'), 'cfg')
print(config_dir)

# Parameters file, see https://github.com/ros2/launch_ros/issues/156
params_file = os.path.join(config_dir, 'workaround_params.yaml')
print(params_file)

# Camera calibration file
camera_config = 'file://' + os.path.join(config_dir, 'my_camera.ini')
print(camera_config)


def generate_launch_description():

    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gscam2',
                plugin='gscam2::GSCamNode',
                name='gscam_publisher',
                namespace=camera_name,
                parameters=[
                    # Some parameters from a yaml file
                    params_file,
                    # A few more parameters
                    {
                        'camera_name': camera_name,  # Camera Name
                        'camera_info_url': camera_config,  # Camera calibration information
                    },
                ],
                # Remap outputs to the correct namespace
                remappings=[
                    ('/image_raw', '/' + camera_name + '/image_raw'),
                    ('/camera_info', '/' + camera_name + '/camera_info'),
                ],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
