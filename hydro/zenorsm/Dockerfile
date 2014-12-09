# docker build -t opencog/ros-hydro-zenorsm .
# xhost +
# docker run --rm --privileged -i -v /dev/ttyACM0:/dev/ttyACM0 -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -v /dev/video0:/dev/video0 -v /dev/video1:/dev/video1 -e DISPLAY=:0.0 -p 33433:33433/udp -t opencog/ros-hydro-zenorsm 
# Note: port 33433=faceshift, /dev/ttyACM0=pololu, /dev/dri=gpu, /dev/shm=gpu
# ToDo: add MORSE robot simulator
#       replace faceshift with OpenCV FaceRecognizer, etc.

FROM opencog/ros-hydro-dev
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get -y update
# Crap, didn't notice that it's installing blender...I think this can be removed? - AvdP
RUN apt-get -y install blender

RUN apt-get -y install python3-yaml 
RUN apt-get -y install python3-setuptools
RUN easy_install3 pip
RUN pip3 install rospkg catkin_pkg

RUN echo source /opt/ros/hydro/setup.bash >> /.bash_profile && \
    echo -e "\e[1;34m[$SELF_NAME] catkin devel setup\.bash sourced\e[0m"

# Install ITFTalk and prerequisites
WORKDIR /catkin_ws/src
RUN apt-get -y install mplayer python-pycurl libcurl3
RUN git clone https://github.com/Alex-van-der-Peet/ITFTalk

# Install ITFListen and prerequisites
WORKDIR /catkin_ws/src
RUN apt-get -y install sox python-simplejson
RUN git clone https://github.com/Alex-van-der-Peet/ITFListen

# Install ITFParrot to test ITFListen -> ITFTalk
WORKDIR /catkin_ws/src
RUN git clone https://github.com/Alex-van-der-Peet/ITFParrot

# I figured it'd be undesirable to auto-start the parrot, so
# to Run ITFParrot, go to /catkin_ws and then run:
# source devel/setup.bash
# rosrlaunch itf_parrot itf_parrot.launch

# Install openEAR prerequisites
RUN apt-get -y install autoconf libtool g++ libportaudio2

# Download ROS-enabled version of openEAR
# WORKDIR /
# Line below is needed otherwise it keeps saying ros_open_ear exists
# and is not empty
# RUN rm -rf /ros_open_ear
# RUN git clone https://github.com/Alex-van-der-Peet/ros_open_ear

# Unpack, make and install portaudio (openEAR prerequisite)
# WORKDIR /ros_open_ear/thirdparty
# RUN tar -xvf portaudio.tgz
# WORKDIR /ros_open_ear/thirdparty/portaudio
# RUN ./configure
# RUN make
# RUN make install

# Configure, build and install the modified version of openEAR
# WORKDIR /ros_open_ear
# Double run below is on purpose
#### ENV C_INCLUDE_PATH /opt/ros/hydro/include/ros:/opt/ros/hydro/include
# RUN ./autogen.sh
# RUN ./autogen.sh
# RUN ./configure
# RUN apt-get install libc6-dev build-essential
# RUN echo "C_INCLUDEPATH=$C_INCLUDE_PATH"
# RUN make
# RUN make install

WORKDIR /catkin_ws/src
RUN cp /opt/ros/hydro/setup.sh /etc/profile.d/ros_hydro.sh
ENV PYTHONPATH /opt/ros/hydro/lib/python2.7/dist-packages
RUN /usr/bin/python3 /opt/ros/hydro/bin/catkin_init_workspace

#Download and install DepthSenseSDK
WORKDIR /Downloads
ADD http://buildbot.opencog.org/hidden/DepthSenseSDK-1.4.3-1527-amd64.deb /Downloads/DepthSenseSDK-1.4.3-1527-amd64.deb
RUN dpkg -i DepthSenseSDK-1.4.3-1527-amd64.deb

WORKDIR /catkin_ws/src

# The steps below clone ROS packages, but since the Gizmo runs out of memory we have to build 
# intermediately every few packages
RUN git clone https://github.com/hansonrobotics/robo_blender
RUN git clone https://github.com/yantrabuddhi/ros_pololu_servo
WORKDIR /catkin_ws

RUN bash -l -c "source /opt/ros/hydro/setup.bash; /usr/bin/python3 /opt/ros/hydro/bin/catkin_make"

WORKDIR /catkin_ws/src

# RUN git clone https://github.com/yantrabuddhi/ros_nmpt_saliency
# Mandeep's saliency tracker above doesn't work, so I'm using the one I made
# which is stored in Jamie's repo - Alex
#RUN git clone https://github.com/jdddog/ros_nmpt_saliency
RUN git clone https://github.com/yantrabuddhi/ros_faceshift
RUN git clone https://github.com/yantrabuddhi/simple_face_tracker

WORKDIR /catkin_ws

RUN bash -l -c "source /opt/ros/hydro/setup.bash; /usr/bin/python3 /opt/ros/hydro/bin/catkin_make"

WORKDIR /catkin_ws/src

# Octomap breaks catkin_build, hence disabled:
#RUN git clone https://github.com/OctoMap/octomap
#RUN git clone https://github.com/Alex-van-der-Peet/ITFOpenEar
# Jamie's repo doesn't work, so taking the source repo of his fork
#RUN git clone https://github.com/jdddog/softkinetic
RUN git clone https://github.com/ipa320/softkinetic

WORKDIR /catkin_ws

RUN bash -l -c "source /opt/ros/hydro/setup.bash; /usr/bin/python3 /opt/ros/hydro/bin/catkin_make"

WORKDIR /catkin_ws/src

WORKDIR /catkin_ws
#RUN bash -l -c "source /opt/ros/hydro/setup.bash; /usr/bin/python3 /opt/ros/hydro/bin/catkin_make"
# RUN /opt/ros/hydro/bin/catkin_make

WORKDIR /catkin_ws

RUN apt-get -y install ros-hydro-cv-camera

ENV STARTSCRIPT "\
echo evaluating startup script... &&\
source /catkin_ws/devel/setup.bash &&\
tmux new-session -d 'echo -e \"\e[1;34mroscore\e[0m\" ; roscore' &&\
tmux set -g set-remain-on-exit on &&\
tmux set-option -g set-remain-on-exit on &&\
tmux bind-key R respawn-window &&\
tmux split-window -d -v -p 25 'echo -e \"\e[1;34mros_pololu_servo\e[0m\" ;\
  rosrun ros_pololu_servo ros_pololu_servo_node' &&\
tmux split-window -d -v -p 25 'echo -e \"\e[1;34mblender robo.blend\e[0m\" ;\
  sleep 4 && blender robo.blend' &&\
tmux split-window -d -v -p 25 'echo -e \"\e[1;34mbash shell\e[0m\" ;\
  bash -l -i' &&\
tmux split-window -d -v -p 25 'echo -e \"\e[1;34mros_itf_talk\e[0m\" ;\
  roslaunch itf_talk itf_talk.launch' &&\
tmux split-window -d -v -p 25 'echo -e \"\e[1;34mros_itf_listen\e[0m\" ;\
  roslaunch itf_listen itf_listen.launch' &&\
tmux select-layout even-vertical &&\
tmux attach \
"

# CMD /bin/bash -l -c "eval $STARTSCRIPT"
