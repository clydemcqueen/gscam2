# Smoke test
#
# Terminal 1:
#     docker build --pull --no-cache --build-arg TARGET_ROS_DISTRO=jazzy --tag gscam2:jazzy .
#     docker run -it gscam2:jazzy
#
# Terminal 2:
#     docker container ls     # Get <container_name>
#     docker exec -it <container_name> /bin/bash
#     source /opt/ros/jazzy/setup.bash
#     ros2 topic list
#     ros2 topic hz /image_raw

ARG TARGET_ROS_DISTRO=jazzy

FROM osrf/ros:$TARGET_ROS_DISTRO-desktop

RUN apt-get update && apt-get upgrade -y

RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer-plugins-base1.0-dev \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /work/my_ws/src

ARG TARGET_ROS_DISTRO

COPY . gscam2

WORKDIR /work/my_ws

RUN apt-get update \
  && rosdep update \
  && rosdep install -y --from-paths . --ignore-src

RUN [ "/bin/bash" , "-c" , "\
  source /opt/ros/$TARGET_ROS_DISTRO/setup.bash \
  && colcon build --event-handlers console_direct+" ]

CMD ["/bin/bash", "-c", "source install/local_setup.bash \
  && export GSCAM_CONFIG='videotestsrc pattern=snow ! video/x-raw,width=1280,height=720 ! videoconvert' \
  && ros2 run gscam2 gscam_main"]
