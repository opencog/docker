#! /bin/bash
#
# Run the docker container. Stop any previously running copies.
#
./stop.sh
docker run --name="opencog-indigo-base" -it opencog/ros-indigo-base
