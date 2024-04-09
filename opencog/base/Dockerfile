#
# Images with all the dependencies for development installed.
#
# Steps to build & run:
# 1. docker build --no-cache -t opencog/opencog-deps .
# 2. docker run --rm -it opencog/opencog-deps

# Ubuntu 22.04.1 LTS (Jammy Jellyfish)
ARG VERSION=22.04
FROM ubuntu:${VERSION}

ENV DEBIAN_FRONTEND=noninteractive

# Bump this date, in order to force the apt-update to run again.
# Otherwise, it will use the cached Docker container, which might
# be stale or be missing secruity updates.
ENV LAST_OS_UPDATE 2023-02-07
RUN apt-get update && apt-get -y upgrade

# Install apt utilities
RUN apt-get -y install apt-transport-https software-properties-common \
      sudo wget tzdata apt-utils

# Install ocpkg. Bump the date to get the latest ocpkg installed;
# Otherwise, this docker build will be cached with an old octool.
ENV LAST_OCPKG_UPDATE 2024-04-07

# The OCPKG_URL can be overridden by specifying
# docker build --build-arg OCPKG_URL=https://some.where.else.com/ockpg
ARG GITHUB_NAME="opencog"
ENV GITHUB_NAME ${GITHUB_NAME}
ARG OCPKG_URL=https://raw.githubusercontent.com/${GITHUB_NAME}/ocpkg/master/ocpkg
RUN echo OCPKG_URL=$OCPKG_URL
ADD $OCPKG_URL /tmp/octool
RUN chmod 755 /tmp/octool;  sync;

# Run octool in multiple steps, instead of one big gulp.
# This makes debugging install failures easier.
RUN /tmp/octool -rdv

# -l default is the latest Link Grammar release tarball.
RUN /tmp/octool -v -l default

# Environment Variables
## Set Locale -- Link Grammar expects this.
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
ARG INSTALL_HASKELL="no"
RUN if [ x${INSTALL_HASKELL} = "xyes" ]; then /tmp/octool -s; fi

# For images built on this
ONBUILD USER root
