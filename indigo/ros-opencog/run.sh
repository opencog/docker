#! /bin/bash
#
# Run the docker container. Stop any previously running copies.
#
./stop.sh

docker run --name="cog-ros" -it opencog/ros-indigo-opencog

clear
