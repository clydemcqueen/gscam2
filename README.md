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

## Available Parameters
Format: `param name` type - description (default)

- `gscam_config` string - with the stream config
- `sync_sink` bool - sync wiht the clock (true)
- `preroll` bool - prefill buffers (false)
- `use_gst_timestamps` bool - Use gst time instead of ROS time (false)
- `image_encoding`, string - encoding type (`sensor_msgs::image_encodings::RGB8`)
- `camera_info_url`, string - Location of camera info file
- `camera_name` string - Camera name 
- `frame_id` string - Camera frame id (camera_frame)

## Publishers
- `camera_info`
- `image_raw`
- `image_raw/compressed` - only if image is encoded as a jpeg stream 