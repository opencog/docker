# docker build -t opencog/ros-hydro-openni .
# xhost +
# docker run -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -i -t opencog/ros-hydro-openni /bin/bash
# docker run -t -i --rm --privileged -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /home/mandeep/sharedFolder:/tmp/shared -v /dev/video0:/dev/video0 mandeep/ros-hydro-openni /bin/bash
# from https://github.com/OpenNI/OpenNI

FROM opencog/ros-hydro-dev
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get -y update
RUN apt-get -y install g++ python libusb-1.0-0-dev freeglut3-dev openjdk-7-jdk
RUN apt-get -y install unzip

ADD https://github.com/OpenNI/OpenNI/archive/master.zip /OpenNI.zip
RUN unzip /OpenNI.zip

#RedistMaker needs doxygen or it fails. dum dumb dumb dumb.
#WORKDIR /OpenNI-master/Platform/Linux/CreateRedist
#RUN ./RedistMaker

#WORKDIR /OpenNI-master/Platform/Linux/Redist
#RUN ./install.sh

WORKDIR /OpenNI-master/Platform/Linux/Build
RUN make
RUN make install

CMD /bin/bash
