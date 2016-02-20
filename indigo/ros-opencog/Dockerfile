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
RUN bash -l -c "make -j6"
RUN bash -l -c "make install"

RUN apt-get -y install guile-2.0-dev cython

WORKDIR /opencog/atomspace/build
RUN bash -l -c "cmake .."
RUN bash -l -c "make -j6"
RUN bash -l -c "make install"

# -----------------------------------------------------
# Need wget to download link-grammar source
RUN apt-get -y install wget

# Download the current released version of link-grammar.
# RUN http://www.abisource.com/downloads/link-grammar/current/link-grammar-5*.tar.gz
# The wget tries to guess the correct file to download w/ wildcard
RUN wget -r --no-parent -nH --cut-dirs=2 http://www.abisource.com/downloads/link-grammar/current/

# Unpack the sources, too.
RUN tar -zxf current/link-grammar-5*.tar.gz

# Need the locales for utf8
RUN apt-get install locales
RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen
RUN echo "ru_RU.UTF-8 UTF-8" >> /etc/locale.gen
RUN echo "he_IL.UTF-8 UTF-8" >> /etc/locale.gen
RUN echo "de_DE.UTF-8 UTF-8" >> /etc/locale.gen
RUN echo "lt_LT.UTF-8 UTF-8" >> /etc/locale.gen
RUN echo "fa_IR.UTF-8 UTF-8" >> /etc/locale.gen
RUN echo "ar_AE.UTF-8 UTF-8" >> /etc/locale.gen
RUN echo "tr_TR.UTF-8 UTF-8" >> /etc/locale.gen

# WTF. In debian wheezy, it is enough to just say locale-gen without
# any arguments. But in trusty, we eneed to be explicit.  I'm confused.
# RUN locale-gen
# Note also: under trusty, fa_IR.UTF-8 causes locale-gen to fail, 
# must use the naked  fa_IR
RUN locale-gen en_US.UTF-8 ru_RU.UTF-8 he_IL.UTF-8 de_DE.UTF-8 lt_LT.UTF-8 fa_IR ar_AE.UTF-8 tr_TR.UTF-8



WORKDIR /opencog/opencog/build
RUN bash -l -c "cmake .."
RUN bash -l -c "make -j8"
RUN bash -l -c "make install"

ENTRYPOINT bash -l -c "bash"
