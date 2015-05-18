# Steps:
#
# 1. docker build --no-cache -t opencog/buildbot .
# 2. docker stop buildmaster
# 3. docker rm buildmaster
# 4. Make sure you mount the appropriate host directory for master to /home/buildbot/master
#    using docker's volumes and forward port 8010 on running the container.
#    docker run -d --name buildmaster -v $RELEVANT_DIRECTORY_PATH:/home/buildbot/master\
#    -p 8010:8010 -it opencog/buildbot

FROM ubuntu:14.04

RUN apt-get update ; apt-get -y upgrade ; apt-get -y autoclean
RUN apt-get -y install git python-pip python-dev
RUN pip install buildbot

# Create and switch user. The user is privileged with no password required
# Disbaled temporarily for expediancy
RUN adduser --disabled-password --gecos "Continuese Integration" buildbot
USER buildbot
WORKDIR /home/buildbot

# Configuration
COPY master.cfg master.cfg

# Defaults
## For communicating with buildslaves
EXPOSE 9989
## For webpage
EXPOSE 8010
# This assumes the  $RELEVANT_DIRECTORY_PATH has the necessary files created on
# running `buildbot create-master master`
CMD buildbot start  --nodaemon master
