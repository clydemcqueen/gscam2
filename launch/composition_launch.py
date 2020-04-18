"""
Dynamically compose GSCamNode and ImageSubscriberNode in a component_container.

Limitations of this container:
 -- stdout is not set to flush after each line, so the most recent log messages may be delayed
"""

import launch
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

# Set your camera namespace
NAMESPACE = "/camera"
# GSCAM config (replace with yours
gscamConfig = 'videotestsrc ! video/x-raw, format=BGRx ! videoconvert'
# Location of config directory (for yamls)
config_dir = os.path.join(get_package_share_directory('your-package'), 'config')
# Get the location of your params file (formatted same as ROS1
paramsFile = os.path.join(config_dir, "params.yaml")
# Get the location of your camera calibration file
cameraConfig = 'file://' + os.path.join(config_dir, "ost.yaml")


def generate_launch_description():


    container = ComposableNodeContainer(
        node_name='my_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gscam',
                node_plugin='gscam::GSCamNode',
                node_name='image_publisher',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='gscam',
                node_plugin='gscam::ImageSubscriberNode',
                node_name='image_subscriber',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Example of remapping and parameter input
            ComposableNode(
                package='gscam',
                node_plugin='gscam::GSCamNode',
                node_name='gscam',
                node_namespace=NAMESPACE,
                # Remap inputs to the correct namespace
                remappings=[
                    ('/image_raw', NAMESPACE + '/image_raw'),
                    ('/camera_info', NAMESPACE + '/camera_info')
                ],
                parameters=[
                    # Use a params file
                    os.path.join(get_package_share_directory('br-perception'), "config", "params.yaml"),
                    # Camera Name
                    {'camera_name': 'right'},
                    # Gscam config
                    {'gscam_config': gscamConfig},
                    # Info URL (Camera calibration information)
                    {'camera_info_url': cameraConfig},
                ]
            )
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])
