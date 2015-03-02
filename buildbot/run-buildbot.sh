#!/bin/bash
#
# This script is run after setting the direcotries for shared filesystem. See 
# README file.

docker run -d --name buildmaster -v $PWD/buildbot_workspace/master:/home/buildbot/master \
    -p 8010:8010 -it opencog/buildbot
docker run -d --name zera --link buildmaster:zera-master \
    -v $PWD/buildbot_workspace/slave:/home/buildslave/slave -it opencog/buildslave:zera

