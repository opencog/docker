ros-indigo-base
===============

The docker image defined here contains only basic ROS nodes shared by
all robots, and nothing more.

## Dependents
* blender contains a docker image for ROS + blender
* dev contains a docker image for a ROS development environment

* animation contains a demo that shows how a blender head can be animated

## Bulding
Use `build-all.sh` to build all the docker images. Use with caution!

## Running
The ros-indigo-base container can be run by saying
`docker run --rm --name="indigo-base" -i -t opencog/ros-indigo-base`

Unlike the blender variants, the base does not require an X11 connection.
See the `blender` and `animation` directories for more examples.

