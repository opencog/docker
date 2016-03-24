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
MAINTAINER Linas Vepštas "linasvepstas@gmail.com"

# Avoid triggering apt-get dialogs (which may lead to errors). See:
# http://stackoverflow.com/questions/25019183/docker-java7-install-fail
ENV DEBIAN_FRONTEND noninteractive

ENV LAST_OS_UPDATE 2016-02-29
RUN apt-get -y update
RUN apt-get -y upgrade

# Need software-properties-common for `add-apt-repository` command
RUN apt-get -y install wget software-properties-common
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get -y update
#
# Most robot bringup will require git-cloning the working repos.
# Need byobu+tmux for ROS terminal control.
# Need vim so that we can do at least some basic debugging.
RUN apt-get -y install ros-indigo-ros-base ros-indigo-angles
RUN apt-get -y install git tmux byobu vim

# Make vim be usable instead of insane.
COPY /scripts/.vimrc /root/

# Environment Variables
## Set Locale -- Catkin crashes on Krisčiunas, Mickevičius without this.
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
CMD /bin/bash
