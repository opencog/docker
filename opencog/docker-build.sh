#!/bin/bash
#
# Notes:
# 1. Build's all the images for development. You can also pull the images from
#    docker registry, but they are a bit bulky.
# 2. If your user is not a member of the docker group you can add it by running
#    sudo adduser $USER docker . On restart you would be able to run docker and
#    this script without root privilege.
# 3. This works for docker version >= 1.5.0 
#
# TODO: Add different options for different usage. For e.g., some person may want
#       to just use the moses image.
#remote_origin basename -s .git $(git remote show origin | grep Fetch | cut -d: -f2-)

# For opencog development
docker build --no-cache -f base/Dockerfile -t opencog/opencog-deps base && \
docker build --no-cache -f tools/cli/Dockerfile -t opencog/opencog-dev:cli tools/cli

# For relex development
docker build --no-cache -f relex/Dockerfile -t opencog/relex relex


