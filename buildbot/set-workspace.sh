#!/bin/bash
#
# This script is used to set a workspace for buildbot to work in.

mkdir buildbot_workspace
cd buildbot_workspace
mkdir master slave

# Configure buildbot master
cd master
docker run --rm --name buildmaster -v $PWD:/home/buildbot/master \
    -p 8010:8010 -it opencog/buildbot buildbot create-master master
printf "Password is required because the container creates the temporary
master.cfg file as root, so replacing it with the master.cfg file
that comes in this repository requires a root privellage."
sudo cp ../../master/master.cfg master.cfg

# Configure buildbot slave
cd ../slave
# Host address added below is just for configuration. On docker run zera-master
# is resolved to master through the container link.
docker run --rm --name zera --add-host zera-master:0.0.0.0 \
    -v $PWD:/home/buildslave/slave -it opencog/buildslave:zera \
    buildslave create-slave slave zera-master zera XXXXXX

