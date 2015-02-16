# docker build -t opencog/ros-indigo-opencog .
# xhost +
# docker run --rm --privileged -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -e DISPLAY=:0.0 -t opencog/ros-indigo-opencog 

FROM opencog/ros-indigo-blender
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get -y update

RUN git clone https://github.com/opencog/opencog
RUN /opencog/scripts/install-dependencies-trusty

WORKDIR /opencog/build
RUN cmake ..
RUN make -j$(ncpus)

CMD /bin/bash
