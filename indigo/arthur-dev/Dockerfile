#
# Container for the Hanson Robotics Arthur head (full version)
# This provides the complete running package for the demo head,
# including UVC webcam vision input, pi_vision to discover faces,
# behavior controled by owyl trees, and the full motor control output.
#
# To build:
# sudo docker build -t opencog/ros-arthur-dev .
#
FROM opencog/ros-indigo-blender
MAINTAINER Vytas Krisciunas "vytas@hansonrobotics.com"
MAINTAINER Linas Vepstas "linasvepstas@gmail.com"

# Install required packages
RUN apt-get -y update
RUN apt-get -y install ros-indigo-openni-camera ros-indigo-mjpeg-server \
          ros-indigo-rosbridge-server ros-indigo-tf ros-indigo-audio-common \
          ros-indigo-driver-common
RUN apt-get -y install python-serial  python-pycurl python-pyglet python-pip
RUN apt-get -y install mplayer
# RUN apt-get -y install libcurl3 libavbin-dev libavbin0 libav-tools

# Install pi_vision
WORKDIR /opt/ros/indigo/share
RUN git clone https://github.com/ericperko/uvc_cam.git
RUN git clone -b multiple_2d https://github.com/hansonrobotics/pi_vision.git
RUN bash -l -c "source /.bashrc;rosmake pi_vision"

# Owyl setup
WORKDIR /opt/
RUN git clone https://github.com/eykd/owyl.git
WORKDIR /opt/owyl/
RUN python setup.py install

# Install catkin packages
WORKDIR /catkin_ws/src

# The basic_head_api repo contains the messages
RUN git clone https://github.com/hansonrobotics/basic_head_api
# The robo_blender contains the head and animation scripts
RUN git clone https://github.com/hansonrobotics/robo_blender
# The pau2motors is where required PAU messages are defined
RUN git clone https://github.com/hansonrobotics/pau2motors

RUN git clone https://github.com/hansonrobotics/ros_pololu_servo.git
RUN git clone https://github.com/hansonrobotics/ros_motors_webui.git
RUN git clone https://github.com/hansonrobotics/robots_config.git
RUN git clone https://github.com/hansonrobotics/chatbot.git
RUN git clone https://github.com/hansonrobotics/eva_behavior.git
RUN git clone https://github.com/hansonrobotics/ros_faceshift.git

# Change line below to rebuild. Will use cache up to this line
ENV LAST_SOFTWARE_UPDATE 2014-12-08

# Git pull for all packages
RUN find . -maxdepth 1 -mindepth 1 -type d \
  -execdir git --git-dir=$PWD/{}/.git --work-tree=$PWD/{} pull \;

# CMake
WORKDIR /catkin_ws
RUN bash -l -c "/usr/bin/python3 /opt/ros/indigo/bin/catkin_make"

RUN echo source /catkin_ws/devel/setup.bash >> ~/.bashrc

#Ports exposed
EXPOSE 9090
EXPOSE 80
EXPOSE 33433

# The launch script
ADD /scripts/arthur-dev.sh /catkin_ws/arthur-dev.sh
RUN chmod ugo+x ./arthur-dev.sh

ENTRYPOINT bash -l -c "cd /catkin_ws/; ./arthur-dev.sh; bash"

