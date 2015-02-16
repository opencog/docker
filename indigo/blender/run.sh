#! /bin/bash
#
# Run the docker container. Stop any previously running copies.
#
./stop.sh
docker run --name="opencog-indigo-blender" \
	--privileged -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 \
	-v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -e DISPLAY=:0.0 \
	-t opencog/ros-indigo-blender

