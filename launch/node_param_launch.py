"""Launch a Node with parameters and remappings."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Your camera namespace
camera_name = 'my_camera'

# Location of configuration directory
config_dir = os.path.join(get_package_share_directory('gscam2'), 'cfg')
print(config_dir)

# Parameters file
params_file = os.path.join(config_dir, 'params.yaml')
print(params_file)

# Camera calibration file
camera_config = 'file://' + os.path.join(config_dir, 'my_camera.ini')
print(camera_config)


def generate_launch_description():

    node = Node(
        package='gscam2',
        executable='gscam_main',
        output='screen',
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
    )

    return LaunchDescription([node])
