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
