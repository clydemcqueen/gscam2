# gscam2 ![ROS2 CI](https://github.com/clydemcqueen/gscam2/actions/workflows/build_test.yml/badge.svg?branch=main)

ROS2 port of [gscam](https://github.com/ros-drivers/gscam).
Supports [ROS2 intra-process comms](https://docs.ros.org/en/humble/Tutorials/Demos/Intra-Process-Communication.html).

> Update 15-Jun-22: [gscam](https://index.ros.org/p/gscam/github-ros-drivers-gscam/) has been released for ROS2. It does not support IPC.

> Update 16-Mar-21: the package name has been updated to match the repository name (gscam2)

## Install and build

Tested on ROS2 Foxy, Galactic, Humble and Rolling.
See the [Dockerfile](Dockerfile) for install and build instructions.

## Usage

Make sure your GStreamer pipeline runs successfully in gst-launch-1.0.
For example, here's a pipeline that works for the [Blue Robotics HD USB Camera](https://bluerobotics.com/store/sensors-sonars-cameras/cameras/cam-usb-low-light-r1/):
~~~
gst-launch-1.0 -v v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! queue ! avdec_h264 ! autovideosink
~~~

Here's the same pipeline in gscam2:
~~~
export GSCAM_CONFIG="v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! avdec_h264 ! videoconvert"
ros2 run gscam2 gscam_main
~~~

Here's an example with parameters:
~~~
ros2 run gscam2 gscam_main --ros-args --remap /image_raw:=/my_camera/image_raw --params-file gscam_params.yaml -p camera_info_url:=file://$PWD/my_camera_info.ini
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
ros2 run gscam2 gscam_main
~~~
There's a [bug](https://github.com/clydemcqueen/gscam2/issues/4) where the last MP4 file is not closed correctly.

### Intra-process comms

IPC test -- CLI composition:
~~~
# First shell
ros2 run rclcpp_components component_container

# Second shell (ignore the deprecation warning, see https://github.com/ros2/ros2cli/issues/336)
ros2 component load /ComponentManager gscam2 gscam2::ImageSubscriberNode -e use_intra_process_comms:=true
ros2 component load /ComponentManager gscam2 gscam2::GSCamNode -e use_intra_process_comms:=true
~~~

Launch file composition:
~~~
ros2 launch gscam2 composition_launch.py
~~~

Manual composition -- handy for debugging:
~~~
ros2 run gscam2 ipc_test_main
~~~

### Finding GStreamer plugins

GStreamer scans various paths for plugins and builds a plugin registry.
[The search process is described here](https://gstreamer.freedesktop.org/documentation/gstreamer/gstregistry.html?gi-language=c).

gscam uses a parameter `gst_plugin_path` instead than the commandline option `--gst-plugin-path`.
The paths in `gst_plugin_path` are searched last, not first.

If you have custom plugins you may need to override the plugin path. Here's an example showing how this works:
~~~
# Disable default locations
export GST_PLUGIN_PATH=""
export GST_PLUGIN_SYSTEM_PATH=""

# Provide the pipeline configuration
export GSCAM_CONFIG=videotestsrc pattern=snow ! video/x-raw,width=1280,height=720 ! videoconvert

# Run gscam_main, providing a custom plugin path
ros2 run gscam2 gscam_main  --ros-args -p gst_plugin_path:="/home/me/myplugins"
~~~

## Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `gst_plugin_path` | string | | Similar to `--gst-plugin-path`, searchs path for plugins |
| `gscam_config` | string | | GStreamer pipeline configuration |
| `sync_sink` | bool | True | Enable GstBaseSink synchronization |
| `preroll` | bool | False | Transition to GST_STATE_PLAYING twice |
| `use_gst_timestamps` | bool | False | Use gst time instead of ROS time |
| `image_encoding` | string | `sensor_msgs::image_encodings::RGB8` |  ROS image encoding |
| `camera_info_url` | string | | URL to camera info file, e.g., `file:///path/to/file` |
| `camera_name` | string | | Replaces `${NAME}` in the URL  |
| `frame_id` | string | camera_frame | Camera frame ID |

## Topics
- `camera_info`
- `image_raw`
- `image_raw/compressed` - only if image is encoded as a jpeg stream 

## Camera info file formats

Uses the [ROS standard camera calibration formats](http://wiki.ros.org/camera_calibration_parsers?distro=melodic).
Files must end in `.ini` or `.yaml`.


