# docker build -t $USER/ros-indigo-blender .
# xhost +
# docker run --rm --privileged -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -e DISPLAY=:0.0 -t dhart/ros-indigo-blender /usr/bin/blender

FROM dhart/ros-indigo-dev
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"

RUN apt-get -y update
RUN apt-get -y install blender

CMD /bin/bash
