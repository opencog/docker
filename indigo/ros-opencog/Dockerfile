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
# Base stuff for cogutil. All on one line, to avoid error
# `Cannot create container with more than 127 parents`
RUN apt-get -y install gcc g++ cmake binutils-dev libiberty-dev \
    libboost-dev libboost-date-time-dev libboost-filesystem-dev \
    libboost-program-options-dev libboost-regex-dev \
    libboost-serialization-dev libboost-system-dev libboost-thread-dev \
    cxxtest

# Additional stuff needed to build and run relex.
RUN apt-get -y install wordnet-dev wordnet-sense-index openjdk-7-jdk \
    ant libcommons-logging-java libgetopt-java

# Additional stuff for the AtomSpace: cython and guile and ODBC.
# Additional stuff for OpenCog. We need telnet as a debug utility.
# Need wget to download link-grammar source.
# Need unzip to unzip JWNL.
RUN apt-get -y install guile-2.0-dev cython unixodbc-dev odbc-postgresql \
    wget telnet locales rlwrap unzip

# -------------------------------------------------------------
# Right now, just clone the OpenCog sources. They will be built
# later on.
WORKDIR /opencog
RUN git clone https://github.com/opencog/cogutil.git
RUN git clone https://github.com/opencog/atomspace.git
RUN git clone https://github.com/opencog/opencog.git
RUN git clone https://github.com/opencog/relex.git

# -------------------------------------------------------------
# Download and install JWNL; its needed for relex.
WORKDIR /src
RUN wget http://downloads.sourceforge.net/project/jwordnet/jwnl/JWNL%201.4/jwnl14-rc2.zip
RUN (unzip jwnl14-rc2.zip jwnl14-rc2/jwnl.jar ; \
    mkdir /usr/local/share/java ; \
    mv -v jwnl14-rc2/jwnl.jar /usr/local/share/java ; \
    rm -v jwnl14-rc2.zip ; rmdir jwnl14-rc2 ; \
    chmod -v 0644 /usr/local/share/java/jwnl.jar)

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
RUN (echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
     echo "ru_RU.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "he_IL.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "de_DE.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "lt_LT.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "fa_IR.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "ar_AE.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "kk_KZ.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "tr_TR.UTF-8 UTF-8" >> /etc/locale.gen)

# WTF. In debian wheezy, it is enough to just say locale-gen without
# any arguments. But in trusty, we eneed to be explicit.  I'm confused.
# RUN locale-gen
# Note also: under trusty, fa_IR.UTF-8 causes locale-gen to fail,
# must use the naked  fa_IR
# Note also: Kazakh is kk_KZ not kz_KZ
RUN locale-gen en_US.UTF-8 ru_RU.UTF-8 he_IL.UTF-8 \
     de_DE.UTF-8 lt_LT.UTF-8 fa_IR ar_AE.UTF-8 kk_KZ.UTF-8 tr_TR.UTF-8

# Build the libraries and command-line parser only
# Assumes that the sources have already been unpacked.
RUN (cd link-grammar-5*; mkdir build; cd build; \
     ../configure; make -j12; make install; ldconfig)

RUN adduser --disabled-password --gecos "Link Parser User" link-parser

USER link-parser
RUN (cd /home/link-parser; echo "export LANG=en_US.UTF-8" >> .bash_aliases)
CMD bash

RUN export LANG=en_US.UTF-8

# -------------------------------------------------------------
USER root
WORKDIR /opencog

# Change line below on rebuild. Will use Docker cache up to this line,
# twigging this date will update the opencog sources.
ENV LAST_SOFTWARE_UPDATE 2016-03-10

# Git pull for all packages
RUN cd /opencog/ && find . -maxdepth 1 -mindepth 1 -type d \
	-execdir git --git-dir=$PWD/{}/.git --work-tree=$PWD/{} pull \;

# Build the base software.
RUN (mkdir /opencog/cogutil/build; cd /opencog/cogutil/build; \
	cmake ..; make -j6; make install)
RUN (mkdir /opencog/atomspace/build; cd /opencog/atomspace/build; \
	cmake ..; make -j12; make install)
RUN (mkdir /opencog/opencog/build; cd /opencog/opencog/build; \
	cmake ..; make -j12; make install)
RUN (cd /opencog/relex; ant build; ant install)

# Build the unit tests. This is .. optional, but we do it anyway,
# for right now, just to sanity-check things.
# WORKDIR /opencog/cogutil/build
# RUN bash -l -c "make -j6 tests"
#
# WORKDIR /opencog/atomspace/build
# RUN bash -l -c "make -j12 tests"
#
# WORKDIR /opencog/opencog/build
# RUN bash -l -c "make -j12 tests"
#
COPY /scripts/.guile /root/
COPY /scripts/unit-test.sh /root/
COPY /scripts/tmux.sh /root/

# Pre-compile the guile modules. We could say `guild comple ...`, but
# this is easier. This avoids fly compilation during container start.
RUN (bash -l -c "echo \"(use-modules (opencog))\" | guile ; \
  echo \"(use-modules (opencog exec))\" | guile ; \
  echo \"(use-modules (opencog logger))\" | guile ; \
  echo \"(use-modules (opencog persist))\" | guile ; \
  echo \"(use-modules (opencog persist-sql))\" | guile ; \
  echo \"(use-modules (opencog query))\" | guile ; \
  echo \"(use-modules (opencog rule-engine))\" | guile ; \
  echo \"(use-modules (opencog atom-types))\" | guile ; \
  echo \"(use-modules (opencog cogserver))\" | guile ; \
  echo \"(use-modules (opencog nlp))\" | guile ; \
  echo \"(use-modules (opencog nlp chatbot))\" | guile ; \
  echo \"(use-modules (opencog nlp chatbot-eva))\" | guile ; \
  echo \"(use-modules (opencog nlp fuzzy))\" | guile ; \
  echo \"(use-modules (opencog nlp lg-dict))\" | guile ; \
  echo \"(use-modules (opencog nlp microplanning))\" | guile ; \
  echo \"(use-modules (opencog nlp relex2logic))\" | guile ; \
  echo \"(use-modules (opencog nlp sureal))\" | guile ")

WORKDIR /root
# ENTRYPOINT bash -l -c "./unit-test.sh; bash"
ENTRYPOINT bash -l -c "./tmux.sh; bash"
# CMD /bin/bash
