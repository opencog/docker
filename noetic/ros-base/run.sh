#! /bin/bash
#
# Run the docker container. Stop any previously running copies.
#
./stop.sh

# Just for fun, we will export the roscore port 11311 out of the
# container.   That way, if you run roscore inside the container,
# and you can `rostopic list` outside of it!
docker run --name="ros-base" \
    -p 11311:11311 -it opencog/ros-noetic-base
