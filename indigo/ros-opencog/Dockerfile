#
# Container for ROS+OpenCog
#
# To build:
# sudo docker build -t opencog/ros-indigo-opencog .
#
FROM opencog/ros-indigo-base
MAINTAINER Linas Vepštas "linasvepstas@gmail.com"

# Install required packages
ENV LAST_OS_UPDATE 2016-02-18
RUN apt-get -y update
RUN apt-get -y upgrade
# Base stuff for cogutils
RUN apt-get -y install gcc g++ cmake binutils-dev libiberty-dev
RUN apt-get -y install libboost-dev libboost-date-time-dev
RUN apt-get -y install libboost-filesystem-dev libboost-program-options-dev
RUN apt-get -y install libboost-regex-dev libboost-serialization-dev
RUN apt-get -y install libboost-system-dev libboost-thread-dev
# Additional stuff for atomspace.
# RUN apt-get -y install guile-2.0-dev cython

WORKDIR /opencog
RUN git clone https://github.com/opencog/cogutils.git
RUN git clone https://github.com/opencog/atomspace.git
RUN git clone https://github.com/opencog/opencog.git

# Change line below on rebuild. Will use cache up to this line
ENV LAST_SOFTWARE_UPDATE 2015-02-19

# Git pull for all packages
RUN cd /opencog/ && find . -maxdepth 1 -mindepth 1 -type d \
	-execdir git --git-dir=$PWD/{}/.git --work-tree=$PWD/{} pull \;

WORKDIR /opencog/cogutils/build
RUN bash -l -c "cmake .."
RUN bash -l -c "make -j8"
RUN bash -l -c "make install"

RUN apt-get -y install guile-2.0-dev cython

WORKDIR /opencog/atomspace/build
RUN bash -l -c "cmake .."
RUN bash -l -c "make -j8"
RUN bash -l -c "make install"

# RUN apt-get -y install aguile-2.0-dev cython

WORKDIR /opencog/opencog/build
RUN bash -l -c "cmake .."
RUN bash -l -c "make -j8"
RUN bash -l -c "make install"

ENTRYPOINT bash -l -c "bash"
