opencog-ros
===========

Dockerfiles for Robot Operating System (ROS), integrated with Blender, Pololu,
Nick's Machine Perception Saliency, and other add-ons, in preparation for
integrating with OpenCog.

Docker image structure:

    ├─ros-hydro-deps
      ├─ros-hydro-dev
        ├─ros-hydro-openni
          ├─ros-hydro-percept

    ├─ros-indigo-base
      ├─ros-indigo-blender
      ├─ros-indigo-dev
        ├─ros-indigo-einstein
        ├─ros-indigo-opencog
        ├─ros-indigo-zenorsm

Images available at https://index.docker.io/u/opencog 

Pull using, e.g., 'docker pull ros-indigo-opencog'

* Design Notes

** ros-indigo-base provides a base set of ROS packages, nothing more.
   The packages are sufficient for performing ROS demos, but no actual
   development.

** ros-indigo-blender adds blender to the base, thus allowing ROS nodes
   to control blender animations.

** ros-indigo-dev provides additional development packages, allowing
   developers to build and test inside of docker containers.

