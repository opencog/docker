#
# Docker file for base ROS+Blender install
# No ROS nodes are started.
#
# The primary difficulty here is that Blender wants Python3
# while ROS wants Python2, and so there are install conflicts.
#
# To build:
# docker build -t opencog/ros-noetic-blender .
#
# To run:
# docker run --rm --privileged -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 \
#    -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -e DISPLAY=:0.0 \
#    -t opencog/ros-noetic-blender
#
# The above should be sufficient to get blender running with full
# 3D hardware acceleration on the local host (dri==direct rendering interface)
# Note: /dev/dri=gpu, /dev/shm=gpu
#
FROM opencog/ros-noetic-base
MAINTAINER Mark Wigzell "markwigzell@gmail.com"

ENV LAST_OS_UPDATE 2020-04-23


# Blender with dependencies
# Install x11-utils to get xdpyinfo, for X11 display debugging
# mesa-utils provides glxinfo, handy for understanding the 3D support.
# Blender needs python3
RUN apt-get -y install  mesa-utils python3-yaml
ENV LAST_OS_UPDATE 2022-02-25_2
################################################################################
# ROS
RUN apt-get -y install  blender
# We install ROS package and catkin package management
RUN pip3 install rospkg catkin_pkg

WORKDIR /catkin_ws/src
RUN cp /opt/ros/noetic/setup.sh /etc/profile.d/ros_noetic.sh
RUN bash -l -c "/usr/bin/python3 /opt/ros/noetic/bin/catkin_init_workspace"

# Must run catkin at least once, even if there is nothing to build;
# it creates the needed directory structure and the needed setup.sh.
WORKDIR /catkin_ws
RUN bash -l -c "/opt/ros/noetic/bin/catkin_make"

# Empty ROS Enviroment Setup
# The user is root; so ~/.bash_profile is /root/.bash_profile
RUN echo source /catkin_ws/devel/setup.bash >> ~/.bash_profile && \
    echo -e "\e[1;34m[$SELF_NAME] catkin devel setup\.bash sourced\e[0m"
# ROS end
#############################################################################

ENV DISPLAY :0.0
CMD /bin/bash
