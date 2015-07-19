#!/bin/bash
#
# This script is used to configure the workspace when buildbot master container
# is started.

# This file's name
SELF_NAME=$(basename $0)

# Functions
usage() {
printf "Usage: ./$SELF_NAME [OPTIONS]

  OPTIONS:
    -c Configure workspace for buildbot master & slaves.
    -r Run buildbot master pro
    -h This help message. \n"
}

configure_workspace() {
# Workspace for master
printf "\n\n----Starting configuring workspace for master"
cp -u master.cfg /var/workspace/master/
buildbot create-master -r /var/workspace/master/
printf "\n\n----Finished configuring workspace for master"

# Workspace for slaves
## zera
printf "\n\n----Starting configuring workspace for builders\n"
buildslave create-slave -r /var/workspace/slaves/zera buildmaster zera XXXXXX
printf "\n\n----Finished configuring workspace for builders"

}

# Main Execution
if [ $# -eq 0 ] ; then NO_ARGS=true ; fi

while getopts "hrc" flag ; do
    case $flag in
        c) CONFIGURE_WORKSPACE=true ;;
        r) RUN_MASTER=true ;;
        h) usage ;;
        \?) usage ;;
        *) UNKNOWN_FLAGS=true ;;
    esac
done

if [ $CONFIGURE_WORKSPACE ] ; then
    configure_workspace
fi

if [ $RUN_MASTER ] ; then
    printf "\n\n----Starting buildbot master daemon"
    buildbot start  --nodaemon /var/workspace/master
fi

if [ $UNKNOWN_FLAGS ] ; then usage ; fi
if [ $NO_ARGS ] ; then usage ; fi
