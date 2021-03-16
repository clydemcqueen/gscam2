"""
Dynamically compose GSCamNode and ImageSubscriberNode in a component_container.
Limitations of this container:
 -- stdout is not set to flush after each line, so the most recent log messages may be delayed
"""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


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
                name='image_publisher',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='gscam2',
                plugin='gscam2::ImageSubscriberNode',
                name='image_subscriber',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])