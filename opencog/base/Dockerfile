#
# Images with all the dependencies for development installed.
#
# Steps to build & run:
# 1. docker build --no-cache -t opencog/opencog-deps .
# 2. docker run --rm -it opencog/opencog-deps

# Default to Ubuntu 24.04 LTS (Noble Numbat)
ARG BASE_OS=ubuntu:24.04
FROM ${BASE_OS}

ENV DEBIAN_FRONTEND=noninteractive

# Bump this date, in order to force the apt-update to run again.
# Otherwise, it will use the cached Docker container, which might
# be stale or be missing security updates.
ENV LAST_OS_UPDATE=2025-11-20
RUN apt-get update && apt-get -y upgrade

# Install apt utilities
RUN apt-get -y install apt-transport-https sudo wget tzdata apt-utils

# Debian does not have software-properties-common; this is an ubuntu pkg
RUN apt-get -y --ignore-missing install software-properties-common || true

# This container is used by circle-ci unit tests, and they seem to
# be happier when ssh is installed. Seems this is NOT installed by
# ocpkg, so we do it here.
RUN apt-get -y install openssh-client

# Install ocpkg. Bump the date to get the latest ocpkg installed;
# Otherwise, this docker build will be cached with an old octool.
# And with the old versions of those packages that octool installs.
ENV LAST_OCPKG_UPDATE=2025-11-20

# The OCPKG_URL can be overridden by specifying
# docker build --build-arg OCPKG_URL=https://some.where.else.com/ockpg
ARG GITHUB_NAME="opencog"
ENV GITHUB_NAME=${GITHUB_NAME}
ARG OCPKG_URL=https://raw.githubusercontent.com/${GITHUB_NAME}/ocpkg/master/ocpkg
RUN echo OCPKG_URL=$OCPKG_URL
ADD $OCPKG_URL /tmp/octool
RUN chmod 755 /tmp/octool;  sync;

# octool -r Add software repositories
#        -d Install base/system build dependencies
#        -v Verbose output for 'apt-get' commands
RUN /tmp/octool -rdv

# Environment Variables

## For using ccache
ENV PATH=/usr/lib/ccache:$PATH

## Set Locale -- Link Grammar expects this.
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# octool -l Install Link Grammar release tarball
ENV LAST_LG_UPDATE=2025-10-28
RUN /tmp/octool -v -l default

# TODO: Remove this user and replace with usage of `-u` flag during runtime.
# Create and switch user. The user is privileged with no password required
RUN adduser --disabled-password --gecos "OpenCog Developer" opencog
RUN adduser opencog sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# video provides access to any GPU's on this device.
RUN usermod -a -G video opencog
USER opencog
WORKDIR /home/opencog

# Setup config files
## Don't mount host volumes to opencog's home directory inside the
## container, if you do you will not find the contents copied below
COPY /scripts/ /home/opencog
RUN sudo chown -R opencog:opencog .guile .gdbinit

# For images built on this
ONBUILD USER root
