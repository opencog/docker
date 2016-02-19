#! /bin/bash
#
# General docker cleanup. Handy if you've been ahcking and need the
# accumulated juk to be blown away.  Use at your own risk.
#
# First, remove the non-running containers.
docker rm $(docker ps -aq -f status=exited)
