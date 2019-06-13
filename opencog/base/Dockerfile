# For creating images with all the dependencies for development installed.
#
# docker build --no-cache -t opencog/opencog-deps .

ARG VERSION=16.04
FROM ubuntu:${VERSION}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y upgrade

# Install tools for developers.
RUN apt-get -y install software-properties-common wget rlwrap telnet \
                       netcat-openbsd less curl vim python3-dbg \
                       tmux man git valgrind gdb sudo byobu apt-utils

# Install repositories and dependencies
ADD https://raw.githubusercontent.com/opencog/ocpkg/master/ocpkg \
    /tmp/octool
RUN apt-get install -y tzdata
RUN chmod 755 /tmp/octool;  sync; /tmp/octool -rdpv -l default

# Environment Variables
## Set Locale
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

## For using ccache
ENV PATH /usr/lib/ccache:$PATH

# Create and switch user. The user is privileged with no password required
RUN adduser --disabled-password --gecos "OpenCog Developer" opencog
RUN adduser opencog sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER opencog
WORKDIR /home/opencog

# Setup the workspace
## Dont't mount host volumes to opencog's home directory inside the container,
## if you do you want find the contents copied below
COPY /scripts/ /home/opencog
RUN sudo chown -R opencog:opencog .*

ARG INSTALL_HASKELL="yes"
RUN if [ ${INSTALL_HASKELL} = "yes" ]; then /tmp/octool -s; fi

# For images built on this
ONBUILD USER root
