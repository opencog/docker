#
# Docker file for base ROS+Blender install
# No ROS nodes are started.
#
# The primary difficulty here is that Blender wants Python3
# while ROS wants Python2, and so there are install conflicts.
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
MAINTAINER Linas Vepštas "linasvepstas@gmail.com"

ENV LAST_OS_UPDATE 2015-11-25

# Get blender version 2.71; the default in trusty is 2.69, and lighting
# looks wrong in it.  The one in ppa:thomas-schiex is 2.75, and gestures
# don't work with that version.
# RUN add-apt-repository ppa:thomas-schiex/blender
RUN add-apt-repository ppa:irie/blender
RUN apt-get -y update
RUN apt-get -y upgrade

# Blender with dependencies
# Install x11-utils to get xdpyinfo, for X11 display debugging
# mesa-utils provides glxinfo, handy for understanding the 3D support.
# Blender needs python3
RUN apt-get -y install blender x11-utils mesa-utils python3-yaml python3-pip

# We perform some magic incantations to allow ROS (which normally
# expects python2) to happily co-exist with python3.  Yes, its odd
# to do this here, but here is where we first install python3.
RUN pip3 install rospkg catkin_pkg

WORKDIR /catkin_ws/src
RUN cp /opt/ros/indigo/setup.sh /etc/profile.d/ros_indigo.sh
ENV PYTHONPATH /opt/ros/indigo/lib/python2.7/dist-packages
RUN /usr/bin/python3 /opt/ros/indigo/bin/catkin_init_workspace

# Must run catkin at least once, even if there is nothing to build;
# it creates the needed directory structure and the needed setup.sh.
WORKDIR /catkin_ws
RUN bash -l -c "/opt/ros/indigo/bin/catkin_make"

# Empty ROS Enviroment Setup
# The user is root; so ~/.bash_profile is /root/.bash_profile
RUN echo source /catkin_ws/devel/setup.bash >> ~/.bash_profile && \
    echo -e "\e[1;34m[$SELF_NAME] catkin devel setup\.bash sourced\e[0m"

ENV DISPLAY :0.0
CMD /bin/bash
