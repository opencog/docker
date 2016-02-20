#
# Container for ROS+OpenCog
#
# To build:
# sudo docker build -t opencog/ros-indigo-opencog .
#
FROM opencog/ros-indigo-base
MAINTAINER Linas Vepštas "linasvepstas@gmail.com"

# Install required packages
ENV LAST_OS_UPDATE 2016-02-20
RUN apt-get -y update
RUN apt-get -y upgrade

WORKDIR /opencog
RUN git clone https://github.com/opencog/cogutils.git

# Change line below on rebuild. Will use cache up to this line
ENV LAST_SOFTWARE_UPDATE 2015-02-19

# Git pull for all packages
RUN cd /opencog/ && find . -maxdepth 1 -mindepth 1 -type d \
	-execdir git --git-dir=$PWD/{}/.git --work-tree=$PWD/{} pull \;

WORKDIR /opencog/cogutils/build
RUN bash -l -c "cmake .."
RUN bash -l -c "make"

ENTRYPOINT bash -l -c "bash"
