#! /bin/bash
#
# Run the docker container. Stop any previously running copies.
#
./stop.sh

# Enable local, non-network connections to X-Server
xhost +local:root

docker run --name="opencog-eva" --privileged  \
   -e DISPLAY=:0.0 \
   -p 33433:33433 -p 80:80 -p 9090:9090 -p 8080:8080 \
   -v /dev/video0:/dev/video0 -v /dev/snd:/dev/snd  \
   -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri \
   -v /dev/shm:/dev/shm -it opencog/eva

xhost -local:root
clear
