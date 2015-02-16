ros-indigo-base
===============

## Heiracrhcy and dependents

* `base` contains a docker image defining only the basic ROS indigo 
  nodes shared by all robots, and nothing more.

* `blender` contains a docker image for ROS and blender. Depends on
   base, above.

* `eva` contains the full end-to-end development environment
   for the Hanson Robotics Eva head.  This includes a half-dozen
   ROS nodes for camera and vision processing, scripted behavior trees,
   motor controllers, and a web-based user interface.

## Miscellaneous packages

* `dev`, derived from `base`, contains a docker image for a ROS indigo
   development environment. (???) (unused ???)

* `einstein` the Handson Robotics Einstein head

* `tracking` .. ??

* ` zenorsm` ... ??

## Unmaintained images
The arthur-animation and arthur-dev packages are deprecated. They
implement an older animation system that was difficult to work with.
Most importantly, eye-tracking and emotion gestures were not blended
and integrated in an easy-to-use fashion.

* `arthur-animation` contains a demo for the Hanson Robotics Arthur head,
   showing how ROS messages can be used to control facial animations.
   That is, the blender rig is encapsulated in a ROS node; the rig
   reacts to ROS messages.

* `arthur-dev` contains the full end-to-end development environment
   for the Hanson Robotics Arthur head.  This includes a half-dozen
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
