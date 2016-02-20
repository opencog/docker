ros-opencog
============

A docker image that installs both ROS and OpenCog.  The image holds
only the base packages needed for running OpenCog and ROS together;
The CogServer is not started, no ROS nodes are started.

## Building

Run the `build.sh` file in this directory.  You need to have built
the base container (in the `../ros-base` directory) first.

## Testing
To verify that OpenCog works, run the `run.sh` shell script.
This will create a tmux (byobu) session inside the docker container,
and run the opencog unit tests in three of the terminals.
