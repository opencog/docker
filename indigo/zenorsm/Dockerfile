# docker build -t opencog/ros-indigo-zenorsm .
# xhost +
# docker run --rm --privileged -i -v /dev/ttyACM0:/dev/ttyACM0 -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -e DISPLAY=:0.0 -p 33433:33433/udp -t opencog/ros-indigo-zenorsm 
# Note: port 33433=faceshift, /dev/ttyACM0=pololu, /dev/dri=gpu, /dev/shm=gpu
# ToDo: add MORSE robot simulator
#       replace faceshift with OpenCV FaceRecognizer, etc.

FROM opencog/ros-indigo-dev
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get -y update
RUN apt-get -y install blender

RUN apt-get -y install python3-yaml python3-pip
RUN pip3 install rospkg catkin_pkg

RUN echo source /catkin_ws/devel/setup.bash >> /.bash_profile && \
    echo -e "\e[1;34m[$SELF_NAME] catkin devel setup\.bash sourced\e[0m"

# Install ITFTalk and prerequisites
WORKDIR /catkin_ws/src
RUN apt-get -y install mplayer python-pycurl libcurl3
RUN apt-get install libavbin-dev libavbin0

RUN git clone https://github.com/geni-lab/ITFTalk

# Install ITFListen and prerequisites
WORKDIR /catkin_ws/src
RUN apt-get -y install sox python-simplejson
RUN git clone https://github.com/geni-lab/ITFListen

# Install ITFParrot to test ITFListen -> ITFTalk
WORKDIR /catkin_ws/src
RUN git clone https://github.com/geni-lab/ITFParrot

# I figured it'd be undesirable to auto-start the aprrot, so
# to Run ITFParrot, go to /catkin_ws and then run:
# source devel/setup.bash
# rosrun itf_parrot itf_parrot.py

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
# ENV C_INCLUDE_PATH /opt/ros/indigo/include/ros:/opt/ros/indigo/include
# RUN ./autogen.sh
# RUN ./autogen.sh
# RUN ./configure
# RUN apt-get install libc6-dev build-essential
# RUN echo "C_INCLUDEPATH=$C_INCLUDE_PATH"
# RUN make
# RUN make install

WORKDIR /catkin_ws/src
RUN cp /opt/ros/indigo/setup.sh /etc/profile.d/ros_indigo.sh
ENV PYTHONPATH /opt/ros/indigo/lib/python2.7/dist-packages
RUN /usr/bin/python3 /opt/ros/indigo/bin/catkin_init_workspace

RUN git clone https://github.com/hansonrobotics/robo_blender
RUN git clone https://github.com/yantrabuddhi/ros_pololu_servo
# RUN git clone https://github.com/yantrabuddhi/ros_nmpt_saliency
# Mandeep's saliency tracker above doesn't work, so I'm using the one I made
# which is stored in Jamie's repo - Alex
RUN git clone https://github.com/jdddog/ros_nmpt_saliency
RUN git clone https://github.com/yantrabuddhi/ros_faceshift
RUN git clone https://github.com/yantrabuddhi/simple_face_tracker
#RUN git clone https://github.com/OctoMap/octomap
#RUN git clone https://github.com/Alex-van-der-Peet/ITFOpenEar

WORKDIR /catkin_ws
RUN bash -l -c "/usr/bin/python3 /opt/ros/indigo/bin/catkin_make"

WORKDIR /catkin_ws/src/robo_blender/src

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
  rosrun itf_talk itf_talk.py' &&\
tmux split-window -d -v -p 25 'echo -e \"\e[1;34mros_itf_listen\e[0m\" ;\
  rosrun itf_listen itf_listen.py' &&\
tmux select-layout even-vertical &&\
tmux attach \
"

CMD /bin/bash -l -c "eval $STARTSCRIPT"
