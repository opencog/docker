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
printf "Usage: ./%s [OPTIONS]

  OPTIONS:
    -b Build opencog/opencog-deps image. It is the base image for
       tools, cogutils, cogserver, and the buildbot images.
    -c Builds opencog/cogutils image. It will build opencog/opencog-deps
       if it hasn't been built, as it forms its base image.
    -m Builds opencog/moses image.
    -p Builds opencog/postgres image.
    -r Builds opencog/relex image.
    -t Builds opencog/opencog-dev:cli image. It will build opencog/opencog-deps
       and opencog/cogutils if they haven't been built, as they form its base
       images.
    -u This option signals all image builds to not use cache.
    -h This help message. \n" "$SELF_NAME"
}

## Build opencog/opencog-deps image.
build_opencog_deps(){
    echo "---- Staring build of opencog/opencog-deps ----"
    docker build $CACHE_OPTION -t opencog/opencog-deps base
    echo "---- Finished build of opencog/opencog-deps ----"
}

## If the opencog/opencog-deps image hasn't been built yet then build it.
check_opencog_deps(){
    if [ -z "$(docker images opencog/opencog-deps | grep -i opencog-deps)" ]
    then build_opencog_deps
    fi
}

## Build opencog/cogutils image.
build_cogutils(){
    check_opencog_deps

    echo "---- Staring build of opencog/cogutils ----"
    docker build $CACHE_OPTION -t opencog/cogutils cogutils
    echo "---- Finished build of opencog/cogutils ----"

}

## If the opencog/cogutils image hasn't been built yet then build it.
check_cogutils(){
    if [ -z "$(docker images opencog/cogutils | grep -i cogutils)" ]
    then build_cogutils
    fi
}

# Main Execution
if [ $# -eq 0 ] ; then NO_ARGS=true ; fi

while getopts "bchmprtu" flag ; do
    case $flag in
        b) BUILD_OPENCOG_BASE_IMAGE=true ;;
        t) BUILD_TOOL_IMAGE=true ;;
        c) BUILD_COGUTILS_IMAGE=true ;;
        m) BUILD__MOSES_IMAGE=true ;;
        p) BUILD__POSTGRES_IMAGE=true ;;
        r) BUILD_RELEX_IMAGE=true ;;
        u) CACHE_OPTION=--no-cache ;;
        h) usage ;;
        \?) usage ;;
        *)  UNKNOWN_FLAGS=true ;;
    esac
done

if [ $BUILD_OPENCOG_BASE_IMAGE ] ; then
    build_opencog_deps
fi

if [ $BUILD_COGUTILS_IMAGE ] ; then
    build_cogutils
fi

if [ $BUILD_TOOL_IMAGE ] ; then
    check_cogutils
    echo "---- Staring build of opencog/opencog-dev:cli ----"
    docker build $CACHE_OPTION -t opencog/opencog-dev:cli tools/cli
    echo "---- Finished build of opencog/opencog-dev:cli ----"
fi

if [ $BUILD__MOSES_IMAGE ] ; then
    echo "---- Staring build of opencog/moses ----"
    docker build $CACHE_OPTION -t opencog/moses moses
    echo "---- Finished build of opencog/moses ----"
fi

if [ $BUILD__POSTGRES_IMAGE ] ; then
    echo "---- Staring build of opencog/postgres ----"
    docker build $CACHE_OPTION -t opencog/postgres postgres
    echo "---- Finished build of opencog/postgres ----"
fi

if [ $BUILD_RELEX_IMAGE ] ; then
    echo "---- Staring build of opencog/relex ----"
    docker build $CACHE_OPTION -t opencog/relex relex
    echo "---- Finished build of opencog/relex ----"
fi

if [ $UNKNOWN_FLAGS ] ; then usage ; fi
if [ $NO_ARGS ] ; then usage ; fi
