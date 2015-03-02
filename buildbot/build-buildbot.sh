#!/bin/bash
#
# This script is used to build the images used for running the buildbot

docker build --no-cache -f master/Dockerfile -t opencog/buildbot master
docker build --no-cache -f slave/latest/Dockerfile -t opencog/buildslave:zera slave/latest
