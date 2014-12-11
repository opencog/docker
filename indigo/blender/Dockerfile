#
# Docker file for base ROS+Blender install
# No ROS nodes are started.
#
# The primary difficulty here is that Blender wants Python3
# while ROS wants Python2, and so there are conflicts.
#
# To build:
# docker build -t opencog/ros-indigo-blender .
#
# To run:
# docker run --rm --privileged -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 \
#    -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -e DISPLAY=:0.0 \
#    -t opencog/ros-indigo-blender
#
# The above should be sufficient to get blender running with full
# 3D hardware acceleration on the local host (dri==direct rendering interface)
# Note: /dev/dri=gpu, /dev/shm=gpu
#
FROM opencog/ros-indigo-base
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"
MAINTAINER Linas Vepstas "linasvepstas@gmail.com"

RUN apt-get -y update
RUN apt-get -y install blender

# Install x11-utils to get xdpyinfo, for X11 display debugging
# mesa-utils provides glxinfo, handy for understanding the 3D support.
ENV DISPLAY :0.0
RUN apt-get -y install x11-utils
RUN apt-get -y install mesa-utils
# Blender needs python3
RUN apt-get -y install python3-yaml python3-pip
RUN pip3 install rospkg catkin_pkg

# The user is root; so ~/.bash_profile is /root/.bash_profile
RUN echo source /catkin_ws/devel/setup.bash >> ~/.bash_profile && \
    echo -e "\e[1;34m[$SELF_NAME] catkin devel setup\.bash sourced\e[0m"

WORKDIR /catkin_ws/src
RUN cp /opt/ros/indigo/setup.sh /etc/profile.d/ros_indigo.sh
ENV PYTHONPATH /opt/ros/indigo/lib/python2.7/dist-packages
RUN /usr/bin/python3 /opt/ros/indigo/bin/catkin_init_workspace

CMD /bin/bash
