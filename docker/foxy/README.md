## Simple test on Foxy

In terminal 1:
~~~
docker build --tag gscam2:foxy .
docker run -it gscam2:foxy
~~~

In terminal 2:
~~~
docker container ls
# Get <container_name>
docker exec -it <container_name> /bin/bash
source /opt/ros/eloquent/setup.bash
ros2 topic list
ros2 topic echo /image_raw
~~~
