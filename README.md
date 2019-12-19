# gscam2

ROS2 port of [gscam](https://github.com/ros-drivers/gscam).
Supports [ROS2 intra-process comms](https://index.ros.org//doc/ros2/Tutorials/Intra-Process-Communication/).

## Install and build

~~~
mkdir ~/ros2/gscam2_ws/src
cd ~/ros2/gscam2_ws/src
git clone https://github.com/clydemcqueen/gscam2.git
git clone https://github.com/ptrmu/ros2_shared.git
cd ~/ros2/gscam2_ws/
source /opt/ros/eloquent/setup.bash
colcon build
~~~
