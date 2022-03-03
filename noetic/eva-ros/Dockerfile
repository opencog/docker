#
# Container for the Hanson Robotics Eva blender rig ROS node, and
# the ROS vision subsystem.  It does not include behaviors or a
# chatbot. On launch, it will start roscore, blender, and the vision
# (face-detection) subsystem, but will not perform any behaviors.
#
# To build:
# sudo docker build -t opencog/eva-ros .
#
FROM opencog/ros-noetic-blender
MAINTAINER Mark Wigzell "markwigzell@gmail.com"

# Install required packages
ENV LAST_OS_UPDATE 2022-02-25
RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get -y install ros-noetic-tf ros-noetic-driver-common
RUN apt-get -y install ros-noetic-cv-bridge ros-noetic-image-transport
RUN apt-get -y install ros-noetic-usb-cam
# The below are needed by perception node
RUN apt-get -y install ros-noetic-robot-state-publisher
RUN apt-get -y install ros-noetic-joint-state-publisher
RUN apt-get -y install ros-noetic-joint-state-publisher-gui

# Install catkinized packages
WORKDIR /catkin_ws/src

RUN git clone https://github.com/opencog/pi_vision.git
RUN git clone https://github.com/opencog/perception.git
RUN git clone https://github.com/opencog/blender_api_msgs.git
RUN git clone https://github.com/opencog/blender_api.git

# The pau2motors package defines the PAU messages, which are
# needed by the perception (vision-geometry) module.
RUN git clone https://github.com/opencog/pau2motors.git
RUN git clone https://github.com/opencog/robots_config.git

# The OpenCog github account mirrors all of the Hanson Robotics
# software.  At this time, the two are in sync with each-other.
# RUN git clone https://github.com/opencog/pi_vision.git
# RUN git clone https://github.com/opencog/perception.git
# RUN git clone https://github.com/opencog/blender_api_msgs.git
# RUN git clone https://github.com/opencog/blender_api.git
# RUN git clone https://github.com/opencog/pau2motors.git
# RUN git clone https://github.com/opencog/robots_config.git

# Change line below on rebuild. Will use cache up to this line
ENV LAST_SOFTWARE_UPDATE 2022-02-25

# Git pull for all packages
RUN cd /catkin_ws/src/ && find . -maxdepth 1 -mindepth 1 -type d \
	-execdir git --git-dir=$PWD/{}/.git --work-tree=$PWD/{} pull \;

# The blender API has not been fully catkinized yet.
RUN pip3 install ./blender_api_msgs/ dlib
# crappy python. Do we have to switch all our calls whenever python has major release?
RUN ln -s /usr/bin/python3 /usr/bin/python

# build the catkin packages. Run twice for good measure. 
# (some may fail first time due to bad deps)
WORKDIR /catkin_ws
RUN bash -l -c "/opt/ros/noetic/bin/catkin_make" || true
RUN bash -l -c "/opt/ros/noetic/bin/catkin_make"

RUN echo source /catkin_ws/devel/setup.bash >> ~/.bashrc

COPY /scripts/eva.sh /catkin_ws/eva.sh
COPY /scripts/sophia.sh /catkin_ws/sophia.sh

# Ports exposed
# 11311 is roscore
EXPOSE 11311

ENTRYPOINT bash -l -c "./sophia.sh; bash"
