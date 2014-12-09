ros-indigo-blender
==================

An integrated demo container for using ROS to control blender animations.
The container holds only the base packages needed for running blender
and ROS together; no actual ROS nodes are started, no blender animations
are run.

* Testing
To verify that blender works, try this:
```
docker run --rm --privileged -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 \
  -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -e DISPLAY=:0.0 \
  -t opencog/ros-indigo-blender 
```

