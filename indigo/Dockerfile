#
# Base ROS indigo installation.
# Bare bones install; does not launch any nodes.
#
# To build:
# docker build -t opencog/ros-indigo-base .
#
# To run:
# docker run --rm --name="indigo-base" -i -t opencog/ros-indigo-base
#
FROM ubuntu:14.04
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"
MAINTAINER Linas Vepstas "linasvepstas@gmail.com"

RUN apt-get -y update
#
RUN apt-get -y install wget
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get -y update
RUN apt-get -y install ros-indigo-ros-base
RUN apt-get -y install ros-indigo-rqt-common-plugins
RUN apt-get -y install ros-indigo-robot ros-indigo-viz
#
# Most robot bringup will require git-cloning the working repos.
# Need byobu+tmux for ROS terminal control.
RUN apt-get -y install git tmux byobu
#
# Why do we think we need these?
# RUN apt-get -y install ros-indigo-bfl ros-indigo-gmapping ros-indigo-laser-pipeline
# RUN apt-get -y install ros-indigo-perception-pcl
# RUN apt-get -y install ros-indigo-perception ros-indigo-simulators

RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
CMD /bin/bash
