#! /bin/bash
#
# run.sh
#
# Usage: ./run.sh
#
# This will start the docker container for the full Arthur head.
# The container can be stopped with ./stop.sh
#
echo -n 'Stopping leftover container.. '
docker stop -t 1 hr-arthur-devel
echo -n 'Removing.. '
docker rm hr-arthur-devel
xhost +
docker run --name="hr-arthur-devel" --privileged  \
   -e DISPLAY=:0.0 \
   -p 33433:33433 -p 80:80 -p 9090:9090 \
   -v /dev/video0:/dev/video0 -v /dev/snd:/dev/snd  \
   -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri \
   -v /dev/shm:/dev/shm -it opencog/ros-arthur-dev
xhost -
