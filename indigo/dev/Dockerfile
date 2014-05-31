# docker build -t $USER/ros-indigo-dev .
# xhost +
# docker run -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -i -t $USER/ros-indigo-dev /bin/bash
# docker run -t -i --rm --privileged -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /home/mandeep/sharedFolder:/tmp/shared -v /dev/video0:/dev/video0 mandeep/ros-indigo-dev /bin/bash

FROM opencog/ros-indigo-deps
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get -y update
RUN apt-get -y install camorama geany
RUN apt-get -y install cmake-curses-gui git mercurial subversion
RUN apt-get -y install less meld vim-gnome git tmux

CMD /bin/bash
