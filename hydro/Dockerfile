# docker build -t opencog/ros-hydro-deps .
# xhost +
# docker run -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -i -t opencog/ros-hydro-deps /bin/bash
# docker run -t -i --rm --privileged -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /home/mandeep/sharedFolder:/tmp/shared -v /dev/video0:/dev/video0 mandeep/ros-hydro-deps /bin/bash
# xhost + is stupid and dangerous, so please run 'xhost -' after launching apps 
# camorama -d /dev/video0
# docker export opencog/ros-hydro | gzip -c > /media/lenovo_mandeep/ros-hydro-deps.tgz
# docker import opencog/ros-hydro < /media/lenovo_mandeep/ros-hydro.tgz
# echo 'DOCKER_OPTS="-H tcp://0.0.0.0:4243 -H unix:///var/run/docker.sock"' \
# | sudo tee -a /etc/default/docker 

FROM ubuntu:12.04
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get -y update
RUN apt-get -y install python-software-properties wget screen
RUN wget http://packages.ros.org/ros.key -O - | apt-key add -
RUN echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get -y update
RUN apt-get -y install ros-hydro-ros-base ros-hydro-rqt-common-plugins
RUN apt-get -y install ros-hydro-robot ros-hydro-viz
RUN apt-get -y install ros-hydro-mobile ros-hydro-perception ros-hydro-simulators

RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
ENV DISPLAY 0:0
CMD /bin/bash
