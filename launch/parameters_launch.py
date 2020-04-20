"""
Dynamically compose GSCamNode and ImageSubscriberNode in a component_container with reammping and parameter
"""

import launch
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

# Set your camera namespace
namespace = "/camera"
# GSCAM config (replace with yours
gscam_config = 'videotestsrc ! video/x-raw, format=BGRx ! videoconvert'
# Location of config directory (for yamls)
config_dir = os.path.join(get_package_share_directory('your-package'), 'config')
# Get the location of your params file (formatted same as ROS1
params_file = os.path.join(config_dir, "params.yaml")
# Get the location of your camera calibration file
camera_config = 'file://' + os.path.join(config_dir, "ost.yaml")


def generate_launch_description():


    container = ComposableNodeContainer(
        node_name='my_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            # Example of remapping and parameter input
            ComposableNode(
                package='gscam',
                node_plugin='gscam::GSCamNode',
                node_name='gscam',
                node_namespace=namespace,
                # Remap inputs to the correct namespace
                remappings=[
                    ('/image_raw', namespace + '/image_raw'),
                    ('/camera_info', namespace + '/camera_info')
                ],
                parameters=[
                    # Use a params file
                    os.path.join(get_package_share_directory('br-perception'), "config", "params.yaml"),
                    # Camera Name
                    {'camera_name': 'right'},
                    # Gscam config
                    {'gscam_config': gscam_config},
                    # Info URL (Camera calibration information)
                    {'camera_info_url': camera_config},
                ]
            )
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])
