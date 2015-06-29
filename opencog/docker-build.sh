#!/bin/bash
#
# Notes:
# 1. Build's all the images for development. You can also pull the images from
#    docker registry, but they are a bit bulky.
# 2. If your user is not a member of the docker group you can add it by running
#    sudo adduser $USER docker . On restart you would be able to run docker and
#    this script without root privilege.
# 3. This works for docker version >= 1.5.0
# 4. If run without -u option it will not rebuild all the images unless the base
#    ubuntu image is updated.

# Environment Variables
## Use's cache by default unless the -u options is passed
CACHE_OPTION=""

## This file/symlinks name
SELF_NAME=$(basename $0)

# Functions
usage() {
printf "Usage: ./$SELF_NAME [OPTIONS]

  OPTIONS:
    -b Build opencog/opencog-deps image. It is the base image for
       tools, cogserver, and the buildbot images.
    -t Builds opencog/opencog-dev:cli image. Also use -b option if you don't
       want to pull from registry.hub.docker.com/u/opencog/opencog-deps/
       or you want to update base image.
    -c Builds opencog/cogserver cogserver image. Also use -b option if you
       don't want to pull from registry.hub.docker.com/u/opencog/opencog-deps/
       or you want to update base image.
    -m Builds opencog/moses image.
    -r Builds opencog/relex image.
    -u This option signals all image builds to not use cache.
    -h This help message. \n"
}

# Main Execution
if [ $# -eq 0 ] ; then NO_ARGS=true ; fi

while getopts "bchmrtu" flag ; do
    case $flag in
        b) BUILD_OPENCOG_BASE_IMAGE=true ;;
        t) BUILD_TOOL_IMAGE=true ;;
        c) BUILD_COGSERVER_IMAGE=true ;;
        m) BUILD__MOSES_IMAGE=true ;;
        r) BUILD_RELEX_IMAGE=true ;;
        u) CACHE_OPTION=--no-cache ;;
        h) usage ;;
        \?) usage ;;
        *)  UNKNOWN_FLAGS=true ;;
    esac
done

if [ $BUILD_OPENCOG_BASE_IMAGE ] ; then
    docker build $CACHE_OPTION -t opencog/opencog-deps base
fi

if [ $BUILD_TOOL_IMAGE ] ; then
    docker build $CACHE_OPTION -t opencog/opencog-dev:cli tools/cli
fi

if [ $BUILD_COGSERVER_IMAGE ] ; then
    docker build $CACHE_OPTION -f cogserver -t opencog/cogserver cogserver
fi

if [ $BUILD__MOSES_IMAGE ] ; then
    docker build $CACHE_OPTION -t opencog/moses moses
fi

if [ $BUILD_RELEX_IMAGE ] ; then
    docker build $CACHE_OPTION -t opencog/relex relex
fi

if [ $UNKNOWN_FLAGS ] ; then usage ; fi
if [ $NO_ARGS ] ; then usage ; fi
