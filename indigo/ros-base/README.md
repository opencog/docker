ros-indigo-base
===============

The base image for building other docker imagess that use ROS. The
container installs and configures a fair variety of ROS packages
needed for running ROS nodes.  No actual ROS nodes are started.

## Building

Run the `build.sh` file in this directory.  You need to have first
obtained (downloaded) the ubuntu:14.04 image from Docker.

## Testing
To verify that things works, run the `run.sh` shell script.
This should result in a shell prompt inside the docker container.
You can start roscore simply by saying `roscore`. Of course, roscore
by itself, does not do anything interesting...
