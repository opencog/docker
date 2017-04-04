FROM gcr.io/tensorflow/tensorflow:latest-gpu

RUN apt-get update

# Install tools for developers.
RUN apt-get -y install sudo rlwrap telnet less tmux man git

# Create and switch user. The user is privileged with no password required
RUN adduser --disabled-password --gecos "OpenCog Developer" opencog
RUN adduser opencog sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER opencog
WORKDIR /home/opencog

COPY .tmux.conf /home/opencog/.tmux.conf

# Docker defaults
CMD bash

# For images built on this
ONBUILD USER root
