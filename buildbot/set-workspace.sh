#!/bin/bash
#
# This script is used to set a workspace for buildbot to work in,
# as well as build the images required.

docker-compose build
mkdir -p buildbot_workspace/master buildbot_workspace/slaves/zera

# Configure buildbot master
docker run --rm \
    -v $PWD/buildbot_workspace/master:/home/buildbot/master \
    buildbot_buildmaster buildbot create-master -r master
#printf "Password is required because the container creates the temporary
#master.cfg file as root, so replacing it with the master.cfg file
#that comes in this repository requires a root privellage."
cp master/master.cfg buildbot_workspace/master/master.cfg

# Configure buildbot slave
## Add as many as you like as long as master.cfg has an entry on how to use
## the builders.
docker run --rm \
    -v $PWD/buildbot_workspace/slaves/zera:/home/buildslave/slave \
    buildbot_zera \
    buildslave create-slave -r slave zera-master zera XXXXXX
