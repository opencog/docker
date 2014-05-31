# docker build -t $USER/ros-indigo-blender .
# xhost +
# docker run --rm --privileged -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -e DISPLAY=:0.0 -t dhart/ros-indigo-blender /usr/bin/blender
# docker run --rm --privileged -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -e DISPLAY=:0.0 -t dhart/ros-indigo-blender /bin/bash -l -c "/opt/ros/indigo/bin/roscore & (sleep 2 ; blender robo.blend)"

FROM opencog/ros-indigo-dev
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get -y update
RUN apt-get -y install blender

RUN apt-get -y install python3-yaml python3-pip
RUN pip3 install rospkg catkin_pkg

WORKDIR /catkin_ws/src
RUN cp /opt/ros/indigo/setup.sh /etc/profile.d/ros_indigo.sh
ENV PYTHONPATH /opt/ros/indigo/lib/python2.7/dist-packages
RUN /usr/bin/python3 /opt/ros/indigo/bin/catkin_init_workspace

RUN git clone https://github.com/hansonrobotics/robo_blender
RUN git clone https://github.com/yantrabuddhi/ros_pololu_servo

WORKDIR /catkin_ws
RUN bash -l -c "/usr/bin/python3 /opt/ros/indigo/bin/catkin_make"
RUN cp /catkin_ws/devel/setup.bash /etc/profile.d/catkin.sh

WORKDIR /catkin_ws/src/robo_blender/src
CMD "roscore & (sleep 2 ; blender robo.blend)"
