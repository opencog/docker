#
# Base ROS noetic installation.
# Bare bones install; does not launch any nodes.
#
# To build:
# docker build -t opencog/ros-noetic-base .
#
# To run:
# docker run --rm --name="noetic-base" -i -t opencog/ros-noetic-base
#
FROM ubuntu:20.04
MAINTAINER Mark Wigzell "markwigzell@gmail.com"

# Avoid triggering apt-get dialogs (which may lead to errors). See:
# http://stackoverflow.com/questions/25019183/docker-java7-install-fail
ENV DEBIAN_FRONTEND noninteractive

ENV LAST_OS_UPDATE 2022-02-25_2
RUN apt-get -y upgrade
RUN apt-get -y update
RUN apt-get -y install iproute2 iputils-ping apt-utils

# Need software-properties-common for `add-apt-repository` command
RUN apt-get -y install wget software-properties-common sudo
RUN apt-get -y  install --no-install-recommends   xserver-xorg-core   xauth   x11-apps x11-utils
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get -y update
#
# Most robot bringup will require git-cloning the working repos.
# Need byobu+tmux for ROS terminal control.
# Need vim so that we can do at least some basic debugging.
RUN apt-get -y install ros-noetic-ros-base ros-noetic-angles
RUN apt-get -y install git tmux byobu vim

# Make vim be usable instead of insane.
COPY /scripts/.vimrc /root/

# Environment Variables
## Set Locale -- Catkin crashes on Krisčiunas, Mickevičius without this.
RUN apt-get -y install locales
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN apt-get -y install python3-pip
RUN pip install -U rosdep
RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
CMD /bin/bash
