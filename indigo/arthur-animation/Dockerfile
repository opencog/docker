#
# Container for the Hanson Robotics Arthur head (demo version)
# This provides just enough infrastructure to run the Arthur rig
# in an animation mode.
#
# To build:
# sudo docker build -t opencog/ros-indigo-animation .
#
# opencog/ros-indigo-blender provides ROS+blender+catkin
FROM opencog/ros-indigo-blender
MAINTAINER Linas Vepstas "linasvepstas@gmail.com"

WORKDIR /catkin_ws/src

# Packages to install
# The basic_head_api repo contains the messages
RUN git clone https://github.com/hansonrobotics/basic_head_api
# The robo_blender contains the head and animation scripts
RUN git clone https://github.com/hansonrobotics/robo_blender
# The pau2motors is where required PAU messages are defined
RUN git clone https://github.com/hansonrobotics/pau2motors

# Change line below to rebuild. Will use cached docker iamges up to
# this line
ENV LAST_SOFTWARE_UPDATE 2014-12-18

# Git pull for all packages
RUN find . -maxdepth 1 -mindepth 1 -type d \
  -execdir git --git-dir=$PWD/{}/.git --work-tree=$PWD/{} pull \;

# Must source and make in same run, else the env is lost!??
WORKDIR /catkin_ws
RUN . /opt/ros/indigo/setup.sh; /opt/ros/indigo/bin/catkin_make

ADD /scripts/arthur.sh /catkin_ws/arthur.sh
ADD /scripts/demo-hints.txt /catkin_ws/demo-hints.txt
RUN chmod ugo+x ./arthur.sh
CMD ./arthur.sh

# CMD /bin/bash
