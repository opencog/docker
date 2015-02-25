# Notes:
# - Tag the image built as opencog/buildbot
# -Make sure you mount the appropriate host directory for master to /home/buildbot/master
#  using docker's volumes and forward port 8010 on running the container.
#  docker run --rm --name master -v $RELEVANT_DIRECTORY_PATH:/home/buildbot/master\
#  -p 8010:8010 -it opencog/buildbot

FROM ubuntu:14.04

RUN apt-get update
RUN apt-get -y install git
RUN apt-get -y install python-pip
RUN apt-get -y install python-dev
RUN pip install buildbot

# Create and switch user. The user is privileged with no password required
# Disbaled temporarily for expediancy
#RUN adduser --disabled-password --gecos "Continuese Integration" buildbot
#USER buildbot
WORKDIR /home/buildbot

# Defaults
EXPOSE 9989
CMD buildbot start master
