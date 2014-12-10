
Facial Expression Animation Demo
================================

Demo of controlling basic facial animations with the Hanson Robotics
Dmitry blender head.  Running this container will start the Dmitry head
in animation mode, and then print a short menu of suggested animations
to try out.  Facial expressions include 'happy', 'sad', 'surprised' and
'angry'. The animations are started and stopped using ROS messages. See
the 'demo-hints.txt' file in this directory for the demo instructions.

Run this image by saying:
```
sudo docker run --rm --privileged -i \
   -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 \
   -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm \
   -e DISPLAY=:0.0 -t opencog/ros-indigo-animation
```

