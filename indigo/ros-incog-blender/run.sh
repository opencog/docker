#! /bin/bash
#
# Run the docker container. Stop any previously running copies.
#
./stop.sh

# Enable local, non-network connections to X-Server
xhost +local:root

docker run --name="opencog-incog-blender" \
	-p 11311:11311 \
	--privileged -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 \
	-v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -e DISPLAY=:0.0 \
	-t opencog/ros-incog-blender

xhost -local:root
