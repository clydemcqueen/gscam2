# gscam2

ROS2 port of [gscam](https://github.com/ros-drivers/gscam).
Supports [ROS2 intra-process comms](https://index.ros.org//doc/ros2/Tutorials/Intra-Process-Communication/).

## Install and build

[Install GStreamer](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html?gi-language=c#),
including this package:
~~~
sudo apt install libgstreamer-plugins-base1.0-dev
~~~

Install this additional ROS package:
~~~
sudo apt install ros-eloquent-camera-info-manager
~~~

Download and build gscam2:
~~~
mkdir ~/ros2/gscam2_ws/src
cd ~/ros2/gscam2_ws/src
git clone https://github.com/clydemcqueen/gscam2.git
git clone https://github.com/ptrmu/ros2_shared.git
cd ~/ros2/gscam2_ws/
source /opt/ros/eloquent/setup.bash
colcon build
source install/local_setup.bash
~~~

## Usage

Make sure your GStreamer pipeline runs successfully in gst-launch-1.0.
For example, here's a pipeline that works for the [Blue Robotics HD USB Camera](https://bluerobotics.com/store/sensors-sonars-cameras/cameras/cam-usb-low-light-r1/):
~~~
gst-launch-1.0 -v v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! queue ! avdec_h264 ! autovideosink
~~~

Here's the same pipeline in gscam2:
~~~
export GSCAM_CONFIG="v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! avdec_h264 ! videoconvert"
ros2 run gscam gscam_main
~~~

Here's an example with parameters:
~~~
ros2 run gscam gscam_main --ros-args --remap /image_raw:=/my_camera/image_raw --params-file gscam_params.yaml -p camera_info_url:=file://$PWD/my_camera_info.ini
~~~
... where gscam_params.yaml is:
~~~
gscam_publisher:
  ros__parameters:
    gscam_config: 'v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! avdec_h264 ! videoconvert'
    preroll: True
    use_gst_timestamps: True
    camera_name: 'my_camera'
    frame_id: 'my_camera_frame'
~~~

Here's an example that uses a GStreamer tee to split the stream, with one stream producing ROS images
and the second stream writing to MP4 files:
~~~
export GSCAM_CONFIG="v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! tee name=fork ! queue ! splitmuxsink location=video%02d.mov max-size-bytes=10000000 fork. ! avdec_h264 ! videoconvert"
ros2 run gscam gscam_main
~~~
There's a [bug](https://github.com/clydemcqueen/gscam2/issues/4) where the last MP4 file is not closed correctly.

### Intra-process comms

IPC test -- CLI composition:
~~~
# First shell
ros2 run rclcpp_components component_container

# Second shell (ignore the deprecation warning, see https://github.com/ros2/ros2cli/issues/336)
ros2 component load /ComponentManager gscam gscam::ImageSubscriberNode -e use_intra_process_comms:=true
ros2 component load /ComponentManager gscam gscam::GSCamNode -e use_intra_process_comms:=true
~~~

Launch file composition:
~~~
ros2 launch gscam composition_launch.py
~~~

Manual composition -- handy for debugging:
~~~
ros2 run gscam ipc_test_main
~~~

## Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `gscam_config` | string | | GStreamer pipeline configuration |
| `sync_sink` | bool | True | Synchronize the app sink |
| `preroll` | bool | False | Prefill buffers |
| `use_gst_timestamps` | bool | False | Use gst time instead of ROS time |
| `image_encoding` | string | `sensor_msgs::image_encodings::RGB8` |  ROS image encoding |
| `camera_info_url` | string | | URL to camera info file, e.g., `file://path/to/file` |
| `camera_name` | string | | Name of the camera, should match the name in the camera info file  |
| `frame_id` | string | camera_frame | Camera frame ID |

## Publishers
- `camera_info`
- `image_raw`
- `image_raw/compressed` - only if image is encoded as a jpeg stream 

## Camera info file formats

Uses the [ROS standard camera calibration formats](http://wiki.ros.org/camera_calibration_parsers?distro=melodic).
Files must end in `.ini` or `.yaml`.


