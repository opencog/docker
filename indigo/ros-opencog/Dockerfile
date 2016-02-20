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
RUN apt-get -y install cxxtest
# Additional stuff for atomspace.
RUN apt-get -y install guile-2.0-dev cython

# Need wget to download link-grammar source
RUN apt-get -y install wget

# -------------------------------------------------------------
# Right now, just clone the OpenCog sources. They will be built
# later on.
WORKDIR /opencog
RUN git clone https://github.com/opencog/cogutils.git
RUN git clone https://github.com/opencog/atomspace.git
RUN git clone https://github.com/opencog/opencog.git

# -------------------------------------------------------------
# Download and build Link Grammar. Its needed for Sureal,
# the language generation component of opencog.
WORKDIR /src

# Touch this tag to grab the latest version of LinkGrammar.
ENV LAST_LG_UPDATE 2016-02-18

# Download the current released version of link-grammar.
# We do NOT want to git-clone this; the git version is unstable;
# it also requires extra tools and mish-mash to build it.
# RUN http://www.abisource.com/downloads/link-grammar/current/link-grammar-5*.tar.gz
# The wget tries to guess the correct file to download w/ wildcard
# But that won't work.
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

# Build the libraries and command-line parser only
# Assumes that the sources have already been unpacked.
RUN (cd link-grammar-5*; mkdir build; cd build; ../configure --disable-java-bindings; make -j12; make install; ldconfig)

RUN adduser --disabled-password --gecos "Link Parser User" link-parser

USER link-parser
WORKDIR /home/link-parser
RUN echo "export LANG=en_US.UTF-8" >> .bash_aliases
CMD bash

RUN export LANG=en_US.UTF-8

# -------------------------------------------------------------
USER root
WORKDIR /opencog

# Change line below on rebuild. Will use Docker cache up to this line,
# twigging this date will update the opencog sources.
ENV LAST_SOFTWARE_UPDATE 2016-02-19

# Git pull for all packages
RUN cd /opencog/ && find . -maxdepth 1 -mindepth 1 -type d \
	-execdir git --git-dir=$PWD/{}/.git --work-tree=$PWD/{} pull \;

# Build the base software.
WORKDIR /opencog/cogutils/build
RUN bash -l -c "cmake .."
RUN bash -l -c "make -j6"
RUN bash -l -c "make install"

WORKDIR /opencog/atomspace/build
RUN bash -l -c "cmake .."
RUN bash -l -c "make -j12"
RUN bash -l -c "make install"

WORKDIR /opencog/opencog/build
RUN bash -l -c "cmake .."
RUN bash -l -c "make -j12"
RUN bash -l -c "make install"

# Build the unit tests. This is .. optional, but we do it anyway,
# for right now, just to sanity-check things.
WORKDIR /opencog/cogutils/build
RUN bash -l -c "make -j6 tests"

WORKDIR /opencog/atomspace/build
RUN bash -l -c "make -j12 tests"

WORKDIR /opencog/opencog/build
RUN bash -l -c "make -j12 tests"

# Run the unit tests, too.
COPY /scripts/unit-test.sh /opencog/unit-test.sh

WORKDIR /opencog
ENTRYPOINT bash -l -c "/opencog/unit-test.sh; bash"
# CMD /bin/bash
