ros-indigo-base
===============

The docker image defined here contains only basic ROS nodes shared by
all robots, and nothing more.

## Dependents

* `dev` contains a docker image for a ROS development environment.

* `blender` contains a docker image for ROS and blender.

* `animation` contains a demo for the Hanson Robotics Arthur head,
   showing how ROS messages can be used to control facial animations.
   That is, the blender rig is encapsulated in a ROS node; the rig
   reacts to ROS messages.

* `arthur-dev` contains the full end-to-end development environment
   for the Hanson Robotics Arthur head.  This includes a half-fozen
   ROS nodes for camera and vision processing, scripted behavior trees,
   motor controllers, and a web-based user interface.

## Bulding
Use `build-all.sh` to build all the docker images. Use with caution!

## Running
The ros-indigo-base container can be run by saying
`docker run --rm --name="indigo-base" -i -t opencog/ros-indigo-base`

Unlike the blender variants, the base does not require an X11 connection.
See the `blender`, `animation` and `arthur-dev` directories for
increasingly complex examples.
