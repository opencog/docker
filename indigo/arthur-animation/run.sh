#! /bin/bash
#
# run.sh
#
# Usage: ./run.sh
#
# This will start the docker container for demoing the Arthur head.
# The container can be stopped with ./stop.sh
#
echo -n 'Stopping leftover container.. '
docker stop -t 1 arthur-animation-demo
echo -n 'Removing.. '
docker rm arthur-animation-demo
xhost +
docker run --name="arthur-animation-demo" --privileged  \
   -e DISPLAY=:0.0 \
   -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri \
   -v /dev/shm:/dev/shm -it opencog/ros-indigo-animation
xhost -
