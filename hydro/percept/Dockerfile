# docker build -t opencog/ros-hydro-percept .
# xhost +
# docker run -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -i -t opencog/ros-hydro-percept /bin/bash
# docker run -t -i --rm --privileged -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /home/mandeep/sharedFolder:/tmp/shared -v /dev/video0:/dev/video0 mandeep/ros-hydro-percept /bin/bash
# from https://github.com/OpenNI/OpenNI

FROM opencog/ros-hydro-openni
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

# RUN apt-get -y update

ADD https://github.com/hansonrobotics/perception_synthesizer/archive/master.zip /perception_synthesizer.zip
RUN unzip /perception_synthesizer.zip

WORKDIR /perception_synthesizer-master/build
RUN cmake ..
RUN make
RUN make install

CMD /bin/bash
