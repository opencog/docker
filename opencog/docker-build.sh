#!/bin/bash
#
# Notes:
# 1. Builds all the images for development. You can also pull the
#    images from docker registry, but they are a bit bulky.
# 2. If your user is not a member of the docker group you can add it
#    by running `sudo adduser $USER docker`. Log out and log back in
#    to get this change. After this is done, you will be able to run
#    this script without root privilege.
# 3. If run without `-u` option, images will not be rebuilt, unless
#    the base ubuntu image is updated.

# Exit on error
set -e

# Environment Variables
## Use cache by default unless the -u options is passed
CACHE_OPTION=""

## This file/symlinks name
SELF_NAME=$(basename $0)
DIR_NAME=$(dirname $0)

# -----------------------------------------------------------------------------
# Main Execution
if [ $# -eq 0 ]; then NO_ARGS=true; fi

while getopts "abcehjlmprstu" flag; do
    case $flag in
    a) PULL_DEV_IMAGES=true ;;
    b) BUILD_OPENCOG_BASE_IMAGE=true ;;
    l) BUILD_LEARN_IMAGE=true ;;
    p) BUILD_ATOMSPACE_PYTHON_IMAGE=true ;;
    s) BUILD_ATOMSPACE_IMAGE=true ;;
    u) CACHE_OPTION=--no-cache ;;
    h) SHOW_USAGE=true ;;
    \?)
        SHOW_USAGE=true
        # exit 1
        ;;
    *) UNKNOWN_FLAGS=true ;;
    esac
done

shift "$(($OPTIND - 1))"

BASE_OS=$1
if [[ -z $BASE_OS ]]; then
    # Default to Ubuntu 24.04 LTS (Noble Numbat)
    # BASE_OS=ubuntu:20.04      # Focal Fossa
    # BASE_OS=ubuntu:22.04      # Jammy Jellyfish
    BASE_OS=ubuntu:24.04      # Noble Numbat
    # BASE_OS=ubuntu:25.10      # Questing Quokka
    # BASE_OS=debian:bullseye   # August 2021
    # BASE_OS=debian:bookworm   # June 2023
    # BASE_OS=debian:trixie     # August 2025
fi
BASE_OS=${BASE_OS//-/:}         # Convert dashes to colons
OS_VERSION=${BASE_OS//:/-}      # Convert colons to dashes

if [[ -z $1 ]]; then            # If net specified, its "latest"
    OS_VERSION=latest           # For dockerhub.io downloads
fi

DOCKER_NAME=$2
if [[ -z $DOCKER_NAME ]]; then
    DOCKER_NAME="opencog"
fi
GITHUB_NAME=$3
if [[ -z $GITHUB_NAME ]]; then
    GITHUB_NAME="opencog"
fi

# Functions
usage() {
    printf "Usage: ./%s [OPTIONS] [BASE_OS] [DOCKER_NAME] [GITHUB_NAME]

  OPTIONS:
    -a Pull all images needed for development from hub.docker.com/u/${DOCKER_NAME}/
    -b Build ${DOCKER_NAME}/opencog-deps:${OS_VERSION} image. Provides
       all dependencies and development tools used by ${DOCKER_NAME}.
    -s Builds ${DOCKER_NAME}/atomspace:${OS_VERSION} image. Builds all
       core AtomSpace packages.
    -p Builds ${DOCKER_NAME}/atomspace-py:${OS_VERSION} image. Adds
       additional node.js and Python packages commonly used in machine
       learning and DL/NN.
    -l Builds ${DOCKER_NAME}/learn:${OS_VERSION} image.

    -u Ignore the docker image cache when building. This will cause the
       container(s) to be built from scratch.
    -h This help message.

  DOCKER_NAME: The name of the Docker user account to be used for this build (default 'opencog').
  GITHUB_NAME: The name of the GitHub user account to be used for this build (default 'opencog') \n" "$SELF_NAME"
}

if [ $SHOW_USAGE ]; then
    usage
    exit 0
fi

# -----------------------------------------------------------------------------
# Sanity check
if ! command -v docker &> /dev/null
then
    echo "Error: docker could not be found!"
    echo "You can fix this on Debian/Ubuntu by saying:"
    echo "$ sudo apt install docker.io"
    exit
fi

# -----------------------------------------------------------------------------
## Shared docker build options

OCPKG_OPTION=""
if [ ! -z "$OCPKG_URL" ]; then
    OCPKG_OPTION="--build-arg OCPKG_URL=$OCPKG_URL"
fi
BASE_OS_OPTION="--build-arg BASE_OS=$BASE_OS"
OS_VERSION_OPTION="--build-arg OS_VERSION=$OS_VERSION"
GITHUB_OPTION="--build-arg GITHUB_NAME=$GITHUB_NAME"

BUILD_OPTIONS="$CACHE_OPTION $BASE_OS_OPTION $OS_VERSION_OPTION $OCPKG_OPTION $GITHUB_OPTION"

# -----------------------------------------------------------------------------
## Build opencog/opencog-deps image.
build_opencog_deps() {
    echo "---- Starting build of ${DOCKER_NAME}/opencog-deps:${OS_VERSION} ----"
    docker build $BUILD_OPTIONS -t ${DOCKER_NAME}/opencog-deps:${OS_VERSION} base
    echo "---- Finished build of ${DOCKER_NAME}/opencog-deps:${OS_VERSION} ----"
}

## If the opencog/opencog-deps image hasn't been built yet then build it.
check_opencog_deps() {
    if [ -z "$(docker images ${DOCKER_NAME}/opencog-deps | grep -i opencog-deps)" ]; then
        build_opencog_deps
    fi
}

# -----------------------------------------------------------------------------
## Build opencog/atomspace image.
build_atomspace() {
    check_opencog_deps
    echo "---- Starting build of ${DOCKER_NAME}/atomspace:${OS_VERSION} ----"
    docker build $BUILD_OPTIONS -t ${DOCKER_NAME}/atomspace:${OS_VERSION} atomspace
    echo "---- Finished build of ${DOCKER_NAME}/atomspace:${OS_VERSION} ----"
}

## If the opencog/atomspace image hasn't been built yet then build it.
check_atomspace() {
    if [ -z "$(docker images ${DOCKER_NAME}/atomspace | grep -i atomspace)" ]; then
        build_atomspace
    fi
}

# -----------------------------------------------------------------------------
## Pull all images needed for development from hub.docker.com/u/opencog/
pull_dev_images() {
    echo "---- Starting pull of opencog development images ----"
    docker pull ${DOCKER_NAME}/opencog-deps
    docker pull ${DOCKER_NAME}/atomspace
    docker pull ${DOCKER_NAME}/learn
    echo "---- Finished pull of opencog development images ----"
}

# -----------------------------------------------------------------------------
# NOTE: To avoid repetion of builds don't reorder the sequence here.

if [ $PULL_DEV_IMAGES ]; then
    pull_dev_images
    exit 0
fi

if [ $BUILD_OPENCOG_BASE_IMAGE ]; then
    build_opencog_deps
fi

if [ $BUILD_ATOMSPACE_IMAGE ]; then
    build_atomspace
fi

if [ $BUILD_ATOMSPACE_PYTHON_IMAGE ]; then
    echo "---- Starting build of ${DOCKER_NAME}/atomspace-py:${OS_VERSION} ----"
    docker build $BUILD_OPTIONS -t ${DOCKER_NAME}/atomspace-py:${OS_VERSION} ${DIR_NAME}/atomspace-py
    echo "---- Finished build of ${DOCKER_NAME}/atomspace-py:${OS_VERSION} ----"
fi

if [ $BUILD_LEARN_IMAGE ] ; then
    echo "---- Starting build of ${DOCKER_NAME}/learn:${OS_VERSION} ----"
    docker build $BUILD_OPTIONS -t ${DOCKER_NAME}/learn:${OS_VERSION} ${DIR_NAME}/learn
    echo "---- Finished build of ${DOCKER_NAME}/learn:${OS_VERSION} ----"
fi

if [ $UNKNOWN_FLAGS ] ; then usage; exit 1 ; fi
if [ $NO_ARGS ] ; then usage ; fi
