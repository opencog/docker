ros-indigo-base
===============
Docker files for Hanson Robotics expressive heads. Currently, the most
sophisticated container here is for the Eva head, which uses a UVC
webcam with scripted behavior trees to drive a blender model of Eva,
a female head that can express a variety of emotional gestures.

## Hierarchy and dependents

* `base` contains a docker image defining only the basic ROS indigo
  nodes shared by all robots, and nothing more.

* `blender` contains a docker image for ROS and blender. Depends on
   base, above.

* `eva` contains the full end-to-end development environment
   for the Hanson Robotics Eva head.  This includes a half-dozen
   ROS nodes for camera and vision processing, scripted behavior trees,
   motor controllers, and a web-based user interface.

* `eva-owyl` contains an older version of the `eva` environment, one
  that does not use OpenCog in any way.  It controls behavior using
  the Owyl behavior tree system.  It does not include any chatbot
  interfaces. It serves as a stable demo of the basic Eva blender
  head, together with the ROS nodes and messaging systems to control it.
  Developmentally, though, this is a dead-end.

## Miscellaneous packages

* `dev`, derived from `base`, contains a docker image for a ROS indigo
   development environment.

* `einstein` container for the Hanson Robotics' small Einstein head.
  This container was demoed at ROS Kong 2014 by David Hanson.

* `zenorsm` ... Zeno ?? with Einstein head ??

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

## Building
Most subdirectories contain a `build.sh` script for building the
particular docker image.

Use `build-all.sh` to build the Eva docker image and it's dependents.
This may take an hour or more.

## Running
Most subdirectories contain a pair of scripts: `run.sh` and `stop.sh`.
These will run and stop the containers defined in that directory.
