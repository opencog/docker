#!/bin/bash
#
# Notes:
# 1. Build's all the images for development. You can also pull the images from
#    docker registry, but they are a bit bulky.
# 2. If your user is not a member of the docker group you can add it by running
#    sudo adduser $USER docker . On restart you would be able to run docker and
#    this script without root privilege.
# 3. This works for docker version >= 1.5.0
# 4. If run without any flag it will not rebuild all the images unless the base
#    ubuntu image is updated.
# 5. If run with the flag --no-cache it will rebuild the whole image irrespective
#    of whether the base ubuntu image was updated or not.
#
# TODO: Add different options for different usage. For e.g., some person may want
#       to just use the moses image.

# For opencog development
docker build $1 -f base/Dockerfile -t opencog/opencog-deps base && \
docker build $1 -f tools/cli/Dockerfile -t opencog/opencog-dev:cli tools/cli && \
docker build $1 -f cogserver -t opencog/cogserver cogserver

# For relex development
docker build $1 -f relex/Dockerfile -t opencog/relex relex
