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

# Exit on error
set -e

# Environment Variables
## Use's cache by default unless the -u options is passed
CACHE_OPTION=""

## This file/symlinks name
SELF_NAME=$(basename $0)

# -----------------------------------------------------------------------------
# Main Execution
if [ $# -eq 0 ]; then NO_ARGS=true; fi

while getopts "abcehjlmprstu" flag; do
    case $flag in
    a) PULL_DEV_IMAGES=true ;;
    b) BUILD_OPENCOG_BASE_IMAGE=true ;;
    c) BUILD_COGUTIL_IMAGE=true ;;
    e) BUILD_EMBODIMENT_IMAGE=true ;;
    j) BUILD_JUPYTER_IMAGE=true ;;
    l) BUILD_LEARN_IMAGE=true ;;
    m) BUILD__MOSES_IMAGE=true ;;
    p) BUILD__POSTGRES_IMAGE=true ;;
    r) BUILD_RELEX_IMAGE=true ;;
    s) BUILD_ATOMSPACE_IMAGE=true ;;
    t) BUILD_TOOL_IMAGE=true ;;
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
DOCKER_NAME=$1
if [[ -z $DOCKER_NAME ]]; then
    DOCKER_NAME="opencog"
fi
GITHUB_NAME=$2
if [[ -z $GITHUB_NAME ]]; then
    GITHUB_NAME="opencog"
fi

# Functions
usage() {
    printf "Usage: ./%s [OPTIONS] [DOCKER_NAME] [GITHUB_NAME]

  OPTIONS:
    -a Pull all images needed for development from hub.docker.com/u/${DOCKER_NAME}/
    -b Build ${DOCKER_NAME}/opencog-deps image. This provides all dependencies
       and development tools used by ${DOCKER_NAME}.
    -c Builds ${DOCKER_NAME}/cogutil image. This is the base image for all
       other ${DOCKER_NAME} images.

    -s Builds ${DOCKER_NAME}/atomspace image.
    -l Builds ${DOCKER_NAME}/learn image.

    -t Builds ${DOCKER_NAME}/opencog-dev image. It contains all supported
       opencog components.
    -j Builds ${DOCKER_NAME}/opencog-jupyter image. It adds a jupyter
       notebook to ${DOCKER_NAME}/opencog-dev:cli

    -u Ignore the docker image cache when building. This will cause the
       container(s) to be built from scratch.
    -h This help message.

Deprecated (Obsolete):
    -e Builds ${DOCKER_NAME}/minecraft image.
    -m Builds ${DOCKER_NAME}/moses image.
    -p Builds ${DOCKER_NAME}/postgres image.
    -r Builds ${DOCKER_NAME}/relex image.

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
    echo "$ sudo apt install docker.io docker-compose"
    exit
fi

# -----------------------------------------------------------------------------
## Build opencog/opencog-deps image.
build_opencog_deps() {
    echo "---- Starting build of ${DOCKER_NAME}/opencog-deps ----"
    OCPKG_OPTION=""
    if [ ! -z "$OCPKG_URL" ]; then
        OCPKG_OPTION="--build-arg OCPKG_URL=$OCPKG_URL"
    fi
    GITHUB_OPTION="--build-arg GITHUB_NAME=$GITHUB_NAME"
    docker build $CACHE_OPTION $OCPKG_OPTION $GITHUB_OPTION -t ${DOCKER_NAME}/opencog-deps base
    echo "---- Finished build of ${DOCKER_NAME}/opencog-deps ----"
}

## If the opencog/opencog-deps image hasn't been built yet then build it.
check_opencog_deps() {
    if [ -z "$(docker images ${DOCKER_NAME}/opencog-deps | grep -i opencog-deps)" ]; then
        build_opencog_deps
    fi
}

# -----------------------------------------------------------------------------
## Build opencog/cogutil image.
build_cogutil() {
    check_opencog_deps
    echo "---- Starting build of ${DOCKER_NAME}/cogutil ----"
    OCPKG_OPTION=""
    if [ ! -z "$OCPKG_URL" ]; then
        OCPKG_OPTION="--build-arg OCPKG_URL=$OCPKG_URL"
    fi
    GITHUB_OPTION="--build-arg GITHUB_NAME=$GITHUB_NAME"
    docker build $CACHE_OPTION $OCPKG_OPTION $GITHUB_OPTION -t ${DOCKER_NAME}/cogutil cogutil
    echo "---- Finished build of ${DOCKER_NAME}/cogutil ----"

}

## If the opencog/cogutil image hasn't been built yet then build it.
check_cogutil() {
    if [ -z "$(docker images ${DOCKER_NAME}/cogutil | grep -i cogutil)" ]; then
        build_cogutil
    fi
}

# -----------------------------------------------------------------------------
## Build opencog/atomspace image.
build_atomspace() {
    check_cogutil
    echo "---- Starting build of ${DOCKER_NAME}/atomspace ----"
    OCPKG_OPTION=""
    if [ ! -z "$OCPKG_URL" ]; then
        OCPKG_OPTION="--build-arg OCPKG_URL=$OCPKG_URL"
    fi
    GITHUB_OPTION="--build-arg GITHUB_NAME=$GITHUB_NAME"
    docker build $CACHE_OPTION $OCPKG_OPTION $GITHUB_OPTION -t ${DOCKER_NAME}/atomspace atomspace
    echo "---- Finished build of ${DOCKER_NAME}/atomspace ----"
}

## If the opencog/atomspace image hasn't been built yet then build it.
check_atomspace() {
    if [ -z "$(docker images ${DOCKER_NAME}/atomspace | grep -i atomspace)" ]; then
        build_atomspace
    fi
}

# -----------------------------------------------------------------------------
## Build opencog/opencog-dev image.
build_dev_cli() {
    check_cogutil
    echo "---- Starting build of ${DOCKER_NAME}/opencog-dev ----"
    GITHUB_OPTION="--build-arg GITHUB_NAME=$GITHUB_NAME"
    docker build $CACHE_OPTION $GITHUB_OPTION -t ${DOCKER_NAME}/opencog-dev tools/cli
    echo "---- Finished build of ${DOCKER_NAME}/opencog-dev ----"
}

## If the opencog/opencog-dev image hasn't been built yet then build it.
check_dev_cli() {
    if [ -z "$(docker images ${DOCKER_NAME}/opencog-dev | grep -i opencog-dev)" ]; then
        build_dev_cli
    fi
}

# -----------------------------------------------------------------------------
## Pull all images needed for development from hub.docker.com/u/opencog/
pull_dev_images() {
    echo "---- Starting pull of opencog development images ----"
    docker pull ${DOCKER_NAME}/opencog-deps
    docker pull ${DOCKER_NAME}/cogutil
    docker pull ${DOCKER_NAME}/atomspace
    docker pull ${DOCKER_NAME}/learn
    docker pull ${DOCKER_NAME}/opencog-dev:cli
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

if [ $BUILD_COGUTIL_IMAGE ]; then
    build_cogutil
fi

if [ $BUILD_ATOMSPACE_IMAGE ]; then
    build_atomspace
fi

if [ $BUILD_LEARN_IMAGE ] ; then
    echo "---- Starting build of ${DOCKER_NAME}/learn ----"
    docker build $CACHE_OPTION -t ${DOCKER_NAME}/learn learn
    echo "---- Finished build of ${DOCKER_NAME}/learn ----"
fi

if [ $BUILD_TOOL_IMAGE ]; then
    build_dev_cli
fi

if [ $BUILD_EMBODIMENT_IMAGE ]; then
    check_dev_cli
    echo "---- Starting build of ${DOCKER_NAME}/minecraft ----"
    docker build $CACHE_OPTION -t ${DOCKER_NAME}/minecraft:0.1.0 minecraft
    echo "---- Finished build of ${DOCKER_NAME}/minecraft ----"
fi

if [ $BUILD__MOSES_IMAGE ]; then
    check_cogutil
    echo "---- Starting build of ${DOCKER_NAME}/moses ----"
    docker build $CACHE_OPTION -t ${DOCKER_NAME}/moses moses
    echo "---- Finished build of ${DOCKER_NAME}/moses ----"
fi

if [ $BUILD__POSTGRES_IMAGE ]; then
    echo "---- Starting build of ${DOCKER_NAME}/postgres ----"
    ATOM_SQL_OPTION=""
    if [ ! -z "$ATOM_SQL_URL" ]; then
        ATOM_SQL_OPTION="--build-arg ATOM_SQL_URL=$ATOM_SQL_URL"
    fi
    docker build $CACHE_OPTION $ATOM_SQL_OPTION -t ${DOCKER_NAME}/postgres postgres
    echo "---- Finished build of ${DOCKER_NAME}/postgres ----"
fi

if [ $BUILD_RELEX_IMAGE ]; then
    echo "---- Starting build of ${DOCKER_NAME}/relex ----"
    RELEX_OPTIONS=""
    if [ ! -z "$RELEX_REPO" ]; then
        RELEX_OPTIONS="--build-arg RELEX_REPO=$RELEX_REPO"
    fi
    if [ ! -z "$RELEX_BRANCH" ]; then
        RELEX_OPTIONS="$RELEX_OPTIONS --build-arg RELEX_BRANCH=$RELEX_BRANCH"
    fi
    docker build $CACHE_OPTION $RELEX_OPTIONS -t ${DOCKER_NAME}/relex relex
    echo "---- Finished build of ${DOCKER_NAME}/relex ----"
fi

if [ $BUILD_JUPYTER_IMAGE ]; then
    check_dev_cli
    echo "---- Starting build of ${DOCKER_NAME}/opencog-jupyter ----"
    docker build $CACHE_OPTION -t ${DOCKER_NAME}/opencog-jupyter tools/jupyter_notebook
    echo "---- Finished build of ${DOCKER_NAME}/opencog-jupyter ----"
fi

if [ $UNKNOWN_FLAGS ] ; then usage; exit 1 ; fi
if [ $NO_ARGS ] ; then usage ; fi
