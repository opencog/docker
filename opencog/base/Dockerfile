# For creating images with all the dependencies for development installed.
#
# docker build --no-cache -t opencog/opencog-deps .

FROM ubuntu:16.04

RUN apt-get update

# Install tools for developers.
RUN apt-get -y install software-properties-common wget rlwrap telnet \
                       netcat-openbsd less curl vim python3-dbg \
                       tmux man git valgrind gdb sudo byobu

# Install repositories and dependencies
ADD https://raw.githubusercontent.com/opencog/ocpkg/master/ocpkg \
    /tmp/octool
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
RUN sudo chown -R opencog:opencog .* ; /tmp/octool -s

# Defaults
## cogserver shell ports
EXPOSE 17001 18001

## REST api
EXPOSE 5000

## ports on which OpenCog's unity3d game communicates with opencog_embodiment
### port from opencog's embodiment code
EXPOSE 16313
### ports from the unity3d game code
EXPOSE 16315 16312

## Default postgresql port
EXPOSE 5432

## Default RelEx & RelEx2Logic port
EXPOSE 4444

## Default ZMQ port
EXPOSE 5563

# Docker defaults
# CMD bash for maintenance only
# ENTRYPOINT should be an application like distcc or buildbot or cogserver
CMD bash

# For images built on this
ONBUILD USER root
