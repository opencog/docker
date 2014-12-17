
Robot Head Development
======================

Full configuration for animating the Hanson Robotics Arthur blender
head.  Running this container will start the UVC video camera drivers,
the pi_vision face discovery module, the eva_behavior module that
implements scrripted behaviors, two blender windows for the Arthur head,
the PAU-2-motor control module, and the Pololu brand motor controllers.
The Web user interfaces is also started.

The behavior of the robot head can be controlled by aiming a web browser
at http://localhost.  This uses the default http port number of 80; this
can be changed in the `run.sh` startup file.

By default, the `/dev/video0` device is used as a webcam for face
tracking.  It can be changed in `run.sh` file.

The container can be started by saying `./run.sh`  and can be halted by
saying `./stop.sh`.

Alternately, you can run this image by saying:
```
xhost +
sudo docker run --rm --privileged -i \
   -p 33433:33433 -p 80:80 -p 9090:9090 \
   -v /dev/video0:/dev/video0 -v /dev/snd:/dev/snd  \
   -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 \
   -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm \
   -e DISPLAY=:0.0 -t opencog/ros-arthur-dev

# When done:
xhost -
```
