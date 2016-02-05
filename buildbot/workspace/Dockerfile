# using opencog's base image b/c one might want compile
FROM ubuntu:14.04

RUN adduser --disabled-password --gecos "OpenCog Developer" opencog
RUN adduser opencog sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Configure Workspace used by buildbot master & slaves
RUN mkdir -p /var/workspace/master /var/workspace/slaves ; \
    chown -R opencog:opencog /var/workspace

USER opencog
WORKDIR /home/opencog


VOLUME /var/workspace
