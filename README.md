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

    ├─ros-indigo-deps
      ├─ros-indigo-dev
        ├─ros-indigo-blender
        ├─ros-indigo-einstein
        ├─ros-indigo-opencog
        ├─ros-indigo-zenorsm

Images available at https://index.docker.io/u/opencog 

Pull using, e.g., 'docker pull ros-indigo-opencog'
