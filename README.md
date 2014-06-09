opencog-ros
===========

Dockerfiles for demoing and working with various different robot heads
and bodies, mostly those from Hanson Robotics.  Several of the heads are
modeled with blender, and so can be usefully worked with and controlled
even without a physical robot.

Currently, the most sophisticated demo here is that of Eva, a female
head created by Hanson Robotics.  She can track human faces visible to
her (via webcam), interact by displaying a variety of emotions and
facial gestures, and perform lip sync for speech.  The Eva blender file
allows all this without the need of a physical robot head to be
available.

Docker is used primarily because there are a large number of software
dependencies that must be installed in order to make this all work.
This includes the Robot Operating System (ROS), Blender, Pololu motor
drivers, a variety of ROS webcam and face-tracking/saliency nodes,
OpenCog, and other add-ons.

Docker image structure:

    ├─ros-hydro-deps
      ├─ros-hydro-dev
        ├─ros-hydro-openni
          ├─ros-hydro-percept

    ├─ros-indigo-base
      ├─ros-indigo-blender
        ├─eva
        ├─ros-arthur-animation
        ├─ros-arthur-dev
        ├─ros-indigo-opencog
      ├─ros-indigo-dev
        ├─ros-indigo-einstein
        ├─ros-indigo-zenorsm

Some (outdated!?) images available at `https://index.docker.io/u/opencog`

Pull using, e.g., `docker pull ros-indigo-opencog`

## Organizational Notes
The base and blender images should be general enough to allow various
different robots to be brought up and demoed.

* `ros-indigo-base` provides a base set of ROS packages, nothing more.
   The packages are sufficient for performing ROS demos, but no actual
   development.

* `ros-indigo-blender` adds blender to the base, thus allowing ROS nodes
   to control blender animations.

* `eva` provides the full Hanson Robotics Eva head demonstration. This
   includes vision and sound processing, motor controls, scripted
   behaviors, and a web user interface.  See the README in `indigo/eva`
   for more details.

* `ros-indigo-dev` provides additional development packages, allowing
   developers to build and test inside of docker containers. XXX
   this needs to be cleaned up and replaced by one of the above!?


### Installation
To run the demos, docker must be installed.  Instructions can be found
here: https://docs.docker.com/installation/ubuntulinux/ .
The *Giving non-root access* section on the page above explains how to
avoid having to use `sudo` all the time.

### Notes
Dockerfiles are static. Some developers use macro expansion via GNU make 
or other tools to make dockerfiles dynamic. For OpenCog's purposes, using
dockerfile inclusion should be sufficient. The Dockerfile at the git root
is self-contained and intended for simply running a cogserver in the most
common configuration, while the dockerfiles here are designed to be built 
in an additive way, all depending on the container tagged opencog-deps in
the same directory as this README.

OpenCog Docker image structure:

    ├─opencog-deps
      ├─opencog-dev (for a dev environment)
      ├─opencog-buildslave
      ├─opencog-distcc
      ├─opencog-embodiment

Using bind mounts (some containers currently use them) is a hacky solution
to passing the opencog source tree to each docker build. Run bindmounts.sh
before running 'docker build' and bindumount.sh to clean up afterward.
