"""Examine how gscam2 and gscam interact with image_transport."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Location of configuration directory
    config_dir = os.path.join(get_package_share_directory('gscam2'), 'cfg')

    # Camera calibration file
    camera_config = 'file://' + os.path.join(config_dir, 'my_camera.ini')

    camera1_nodes = [
        # GSCam2 does not use image_transport and advertises on these topics:
        #       /namespace/camera_info
        #       /namespace/image_raw
        #
        # To use image_transport::republish you'll need to remap topics on gscam_main or republish.
        Node(
            package='gscam2',
            executable='gscam_main',
            output='screen',
            namespace='camera1',
            parameters=[
                {
                    'camera_name': 'camera1',
                    'camera_info_url': camera_config,
                    'gscam_config': 'videotestsrc ! video/x-raw, format=BGRx ! videoconvert',
                    'frame_id': 'my_camera',
                },
            ],
            remappings=[
                ('/camera1/image_raw', '/camera1/in'),
            ]
        ),

        # As configured, this will subscribe to /namespace/in and publish on these topics:
        #       /namespace/out
        #       /namespace/out/compressed
        #       ... additional transports may be installed
        Node(
            package='image_transport',
            executable='republish',
            output='screen',
            namespace='camera1',
            arguments=['raw'],
        ),
    ]

    # GSCam uses image_transport and advertises on these topics:
    #       /namespace/camera/camera_info
    #       /namespace/camera/image_raw
    #       /namespace/camera/image_raw/compressed
    #       ... additional transports may be installed
    #
    # There is no need to use republish.
    camera2_nodes = [
        Node(
            package='gscam',
            executable='gscam_node',
            output='screen',
            namespace='camera2',
            parameters=[
                {
                    'camera_name': 'camera2',
                    'camera_info_url': camera_config,
                    'gscam_config': 'videotestsrc ! video/x-raw, format=BGRx ! videoconvert',
                    'frame_id': 'my_camera',
                },
            ],
        ),
    ]
    return LaunchDescription(camera1_nodes + camera2_nodes)
