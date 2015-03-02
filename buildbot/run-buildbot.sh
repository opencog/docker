#!/bin/bash
#
# This script is run after setting the direcotries for shared filesystem. See 
# README file.
#
# Note that on buildmaster port 8010 is used for the webpage and 9989 is used for
# networking with build slaves.

docker run -d --name buildmaster -v $PWD/buildbot_workspace/master:/home/buildbot/master \
    -p 8010:8010  -p  9989:9989 -it opencog/buildbot
docker run -d --name zera --link buildmaster:zera-master \
    -v $PWD/buildbot_workspace/slave:/home/buildslave/slave -it opencog/buildslave:zera

