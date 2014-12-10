#
# Container for the current dmitry head.
# Uses several soon-to-be-outdated ROS nodes.
#
# To build:
# sudo docker build -t hansonrobotics/dmitry .
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

# Must source and make in same run, else the env is lost!??
WORKDIR /catkin_ws
RUN . /opt/ros/indigo/setup.sh; /opt/ros/indigo/bin/catkin_make

ADD ./dmitry.sh /catkin_ws/dmitry.sh
ADD ./demo-hints.txt /catkin_ws/demo-hints.txt
RUN chmod ugo+x ./dmitry.sh
CMD ./dmitry.sh

# CMD /bin/bash
