#!/bin/bash
#
# This script is a clean-up script to stop and remove running buildbot containers

docker rm -f zera
docker rm -f buildmaster
