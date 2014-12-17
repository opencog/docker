opencog-ros
===========

Dockerfiles for Robot Operating System (ROS), integrated with Blender, Pololu,
Nick's Machine Perception Saliency, OpenCog, and other add-ons.

Docker image structure:

    ├─ros-hydro-deps
      ├─ros-hydro-dev
        ├─ros-hydro-openni
          ├─ros-hydro-percept

    ├─ros-indigo-base
      ├─ros-indigo-blender
        ├─ros-indigo-animation
        ├─ros-arthur-dev
      ├─ros-indigo-dev
        ├─ros-indigo-einstein
        ├─ros-indigo-opencog
        ├─ros-indigo-zenorsm

Images available at `https://index.docker.io/u/opencog`

Pull using, e.g., `docker pull ros-indigo-opencog`

## Organizational Notes
The base and blender images should be general enough to allow various
different robots to be brought up and demoed.

* `ros-indigo-base` provides a base set of ROS packages, nothing more.
   The packages are sufficient for performing ROS demos, but no actual
   development.

* `ros-indigo-blender` adds blender to the base, thus allowing ROS nodes
   to control blender animations.

* `ros-indigo-animation` provides a demo of the Hanson Robotics Arthur
   blender rig inside of a ROS node. ROS messages are used to control
   facial expression animations.  See the README in the indigo/animation
   directory for more details.

* `ros-arthur-dev` provides the full Hanson Robotics Arthur head
   development environment. This includes vision and sound processing,
   motor controls, scripted behaviors, and a web user interface.
   See the README in indigo/arthur-dev for more details.

* `ros-indigo-dev` provides additional development packages, allowing
   developers to build and test inside of docker containers. XXX
   this needs to be cleaned up and replaced by one of the above!?


### Installation
To run the demos, docker must be installed.  Instructions can be found
here: https://docs.docker.com/installation/ubuntulinux/ .
The *Giving non-root access* section on the page above explains how to
avoid having to use `sudo` all the time.
