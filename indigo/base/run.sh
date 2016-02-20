#! /bin/bash
#
# Run the docker container. Stop any previously running copies.
#
./stop.sh

# Just for fun, we will export the roscore port out of the container.
# that way, if you run roscore inside the container, you can `rostopic
# list` outside of it!
docker run --name="opencog-indigo-base" \
    --net=host -p 11311:11311 -it opencog/ros-indigo-base
