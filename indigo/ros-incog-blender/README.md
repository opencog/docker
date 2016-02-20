ros-incog-blender
=================

A docker image that installs ROS, OpenCog and blender.  The image holds
only the base packages needed for running OpenCog, blender and ROS
together; no actual ROS nodes are started, the CogServer is not started,
and no blender animations are run.

## Building

Run the `build.sh` file in this directory.  You need to have built
the opencog image (in the `../ros-opencog` directory) first.

## Testing
To verify that blender works, run the `run.sh` shell script.
This should result in a shell prompt inside the docker container.
You can start blender simply by saying `blender`; it should display
on your desktop.
