from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    node_1 = [Node(
            namespace='camera1',
            package='gscam2',
            executable='gscam_main',
            output='screen',
            remappings=[
                ('/image_raw', '/my_camera/image_raw')
            ],
            parameters=[
                {'gscam_config':'udpsrc buffer_size=9216000 port=10000 caps = "application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)RAW,sampling=(string)YCbCr-4:2:2,depth=(string)8,width=(string)1080,height=(string)960,colorimetry=(string)BT601-5,payload=(int)96,a-framerate=(string)60" ! rtpvrawdepay ! videoconvert'},
                {'preroll': False},
                {'use_gst_timestamps': True},
                {'camera_name': 'my_camera1'},
                {'frame_id': 'my_camera_frame'},
                {'camera_info_url': 'file:///home/rgs8805/projects/gscam2_ws/src/gscam2/cfg/my_camera1.ini'},
                {'sync_sink': False},
                ],
            ),
    ]
    node_2 = [Node(
            namespace='camera2',
            package='gscam2',
            executable='gscam_main',
            output='screen',
            remappings=[
                ('/image_raw', '/my_camera/image_raw')
            ],
            parameters=[
                {'gscam_config':'udpsrc buffer_size=9216000 port=10001 caps = "application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)RAW,sampling=(string)YCbCr-4:2:2,depth=(string)8,width=(string)1080,height=(string)960,colorimetry=(string)BT601-5,payload=(int)96,a-framerate=(string)60" ! rtpvrawdepay ! videoconvert'},
                {'preroll': False},
                {'use_gst_timestamps': True},
                {'camera_name': 'my_camera2'},
                {'frame_id': 'my_camera_frame'},
                {'camera_info_url': 'file:///home/rgs8805/projects/gscam2_ws/src/gscam2/cfg/my_camera1.ini'},
                {'sync_sink': False},
                ],
            ),
    ]
    node_3 = [Node(
            namespace='camera3',
            package='gscam2',
            executable='gscam_main',
            output='screen',
            remappings=[
                ('/image_raw', '/my_camera/image_raw')
            ],
            parameters=[
                {'gscam_config':'udpsrc buffer_size=9216000 port=10002 caps = "application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)RAW,sampling=(string)YCbCr-4:2:2,depth=(string)8,width=(string)1080,height=(string)960,colorimetry=(string)BT601-5,payload=(int)96,a-framerate=(string)60" ! rtpvrawdepay ! videoconvert'},
                {'preroll': False},
                {'use_gst_timestamps': True},
                {'camera_name': 'my_camera3'},
                {'frame_id': 'my_camera_frame'},
                {'camera_info_url': 'file:///home/rgs8805/projects/gscam2_ws/src/gscam2/cfg/my_camera1.ini'},
                {'sync_sink': False},
                ],
            ),
    ]
    node_4 = [Node(
            namespace='camera4',
            package='gscam2',
            executable='gscam_main',
            output='screen',
            remappings=[
                ('/image_raw', '/my_camera/image_raw')
            ],
            parameters=[
                {'gscam_config':'udpsrc buffer_size=9216000 port=10003 caps = "application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)RAW,sampling=(string)YCbCr-4:2:2,depth=(string)8,width=(string)1080,height=(string)960,colorimetry=(string)BT601-5,payload=(int)96,a-framerate=(string)60" ! rtpvrawdepay ! videoconvert'},
                {'preroll': False},
                {'use_gst_timestamps': True},
                {'camera_name': 'my_camera4'},
                {'frame_id': 'my_camera_frame'},
                {'camera_info_url': 'file:///home/rgs8805/projects/gscam2_ws/src/gscam2/cfg/my_camera1.ini'},
                {'sync_sink': False},
                ],
            ),
    ]
    node_5 = [Node(
            namespace='camera5',
            package='gscam2',
            executable='gscam_main',
            output='screen',
            remappings=[
                ('/image_raw', '/my_camera/image_raw')
            ],
            parameters=[
                {'gscam_config':'udpsrc buffer_size=9216000 port=10004 caps = "application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)RAW,sampling=(string)YCbCr-4:2:2,depth=(string)8,width=(string)1080,height=(string)960,colorimetry=(string)BT601-5,payload=(int)96,a-framerate=(string)60" ! rtpvrawdepay ! videoconvert'},
                {'preroll': False},
                {'use_gst_timestamps': True},
                {'camera_name': 'my_camera5'},
                {'frame_id': 'my_camera_frame'},
                {'camera_info_url': 'file:///home/rgs8805/projects/gscam2_ws/src/gscam2/cfg/my_camera1.ini'},
                {'sync_sink': False},
                ],
            ),
    ]
    node_6 = [Node(
            namespace='camera6',
            package='gscam2',
            executable='gscam_main',
            output='screen',
            remappings=[
                ('/image_raw', '/my_camera/image_raw')
            ],
            parameters=[
                {'gscam_config':'udpsrc buffer_size=9216000 port=10005 caps = "application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)RAW,sampling=(string)YCbCr-4:2:2,depth=(string)8,width=(string)1080,height=(string)960,colorimetry=(string)BT601-5,payload=(int)96,a-framerate=(string)60" ! rtpvrawdepay ! videoconvert'},
                {'preroll': False},
                {'use_gst_timestamps': True},
                {'camera_name': 'my_camera6'},
                {'frame_id': 'my_camera_frame'},
                {'camera_info_url': 'file:///home/rgs8805/projects/gscam2_ws/src/gscam2/cfg/my_camera1.ini'},
                {'sync_sink': False},
                ],
            ),
    ]

    return LaunchDescription(node_1 + node_2 + node_3 + node_4 + node_5 + node_6)
