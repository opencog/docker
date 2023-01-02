#
# Images with all the dependencies for development installed.
#
# Steps to build & run:
# 1. docker build --no-cache -t opencog/opencog-deps .
# 2. docker run --rm -it opencog/opencog-deps

ARG VERSION=20.04
FROM ubuntu:${VERSION}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y upgrade

# Install apt utilities
RUN apt-get -y install apt-transport-https software-properties-common sudo \
      wget tzdata apt-utils

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

# TODO: Remove this user and replace with usage of `-u` flag during runtime.
# Create and switch user. The user is privileged with no password required
RUN adduser --disabled-password --gecos "OpenCog Developer" opencog
RUN adduser opencog sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER opencog
WORKDIR /home/opencog

# Setup config files
## Dont't mount host volumes to opencog's home directory inside the
## container, if you do you will not find the contents copied below
COPY /scripts/ /home/opencog
RUN sudo chown -R opencog:opencog .guile .gdbinit

# Disable Haskell; its unused, unloved.
# ARG INSTALL_HASKELL="yes"
RUN if [ x${INSTALL_HASKELL} = "xyes" ]; then /tmp/octool -s; fi

# For images built on this
ONBUILD USER root
