ros-noetic-blender
==================

A docker image that installs both ROS and blender, for demoing
ROS-controlled blender animations.  The image holds only the base
packages needed for running blender and ROS together; no actual ROS
nodes are started, no blender animations are run.

## Building

Run the `build.sh` file in this directory.  You need to have built
the base ROS image (in the `../ros-base` directory) first.

## Testing
To verify that blender works, run the `run.sh` shell script.
This should result in a shell prompt inside the docker container.
You can start blender simply by saying `blender`; it should display
on your desktop.
