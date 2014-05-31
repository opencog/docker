# docker build -t opencog/ros-hydro-dev .
# xhost +
# docker run -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -i -t opencog/ros-hydro-dev /bin/bash
# docker run -t -i --rm --privileged -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /home/mandeep/sharedFolder:/tmp/shared -v /dev/video0:/dev/video0 mandeep/ros-hydro-dev /bin/bash

FROM opencog/ros-hydro-deps
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get -y update
RUN apt-get -y install camorama geany
RUN apt-get -y install cmake-curses-gui git mercurial subversion
RUN apt-get -y install ros-hydro-gscam
RUN apt-get -y install less meld vim-gnome

CMD /bin/bash
