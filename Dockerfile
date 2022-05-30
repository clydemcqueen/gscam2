# Smoke test
#
# Terminal 1:
#     docker build --pull --no-cache --build-arg TARGET_ROS_DISTRO=foxy --tag gscam2:foxy .
#     docker run -it gscam2:foxy
#
# Terminal 2:
#     docker container ls     # Get <container_name>
#     docker exec -it <container_name> /bin/bash
#     source /opt/ros/foxy/setup.bash
#     ros2 topic list
#     ros2 topic hz /image_raw

ARG TARGET_ROS_DISTRO=foxy
ARG ROS2_SHARED_BRANCH=master

FROM osrf/ros:$TARGET_ROS_DISTRO-desktop

RUN apt-get update
RUN apt-get upgrade -y

RUN apt-get install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
 gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools \
 gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
 libgstreamer-plugins-base1.0-dev

WORKDIR /work/my_ws/src

ARG TARGET_ROS_DISTRO
ARG ROS2_SHARED_BRANCH

COPY . gscam2

RUN git clone https://github.com/ptrmu/ros2_shared.git -b $ROS2_SHARED_BRANCH

WORKDIR /work/my_ws

RUN rosdep install -y --from-paths . --ignore-src

RUN /bin/bash -c "source /opt/ros/$TARGET_ROS_DISTRO/setup.bash && colcon build"

CMD ["/bin/bash", "-c", "source install/local_setup.bash \
&& export GSCAM_CONFIG='videotestsrc pattern=snow ! video/x-raw,width=1280,height=720 ! videoconvert' \
&& ros2 run gscam2 gscam_main"]
