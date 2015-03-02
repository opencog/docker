# Steps:
#
# 1. docker build --no-cache -t opencog/buildbot .
# 2. docker stop buildmaster
# 3. docker rm buildmaster
# 2. Make sure you mount the appropriate host directory for master to /home/buildbot/master
#    using docker's volumes and forward port 8010 on running the container.
#    docker run -d --name buildmaster -v $RELEVANT_DIRECTORY_PATH:/home/buildbot/master\
    -p 8010:8010 -it opencog/buildbot

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
# This assumes the  $RELEVANT_DIRECTORY_PATH has the necessary files created on
# running `buildbot create-master master`
CMD buildbot start  --nodaemon master
