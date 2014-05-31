# docker build -t $USER/ros-indigo-deps .
# xhost +
# docker run -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -i -t $USER/ros-indigo-deps /bin/bash
# docker run -t -i --rm --privileged -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /home/mandeep/sharedFolder:/tmp/shared -v /dev/video0:/dev/video0 mandeep/ros-indigo-deps /bin/bash
# xhost + is stupid and dangerous, so please run 'xhost -' after launching apps 
# camorama -d /dev/video0
# docker export $USER/ros-indigo | gzip -c > /media/lenovo_mandeep/ros-indigo-deps.tgz
# docker import $USER/ros-indigo < /media/lenovo_mandeep/ros-indigo.tgz
# echo 'DOCKER_OPTS="-H tcp://0.0.0.0:4243 -H unix:///var/run/docker.sock"' \
# | sudo tee -a /etc/default/docker 

FROM ubuntu:14.04
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get -y update
RUN apt-get -y install software-properties-common wget screen
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get -y update
RUN apt-get -y install ros-indigo-ros-base ros-indigo-rqt-common-plugins
RUN apt-get -y install ros-indigo-robot ros-indigo-viz
RUN apt-get -y install ros-indigo-bfl ros-indigo-gmapping ros-indigo-laser-pipeline ros-indigo-perception-pcl ros-indigo-robot
RUN apt-get -y install ros-indigo-perception ros-indigo-simulators

RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
ENV DISPLAY 0:0
CMD /bin/bash
