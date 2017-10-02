#!/bin/bash
#
# This script is used for running the builders. Pass only a single
# Usage: Follow the steps in ../../README.md.

# Functions
usage() {
printf "Usage: ./$SELF_NAME [OPTIONS]

  OPTIONS:
    -a Wait for configuration of workspace and run buildslave process for
       atomspace tests.
    -c Wait for configuration of workspace and run buildslave process for
       cogutil tests.
    -o Wait for configuration of workspace and run buildslave process for
       cogutil tests.
    -m Wait for configuration of workspace and run buildslave process for
       moses tests.
    -h This help message. \n"
}


# Main Execution
ATOMSPACE_WORKSPACE_CONFIGURED=false
COGUTIL_WORKSPACE_CONFIGURED=false
OPENCOG_WORKSPACE_CONFIGURED=false
MOSES_WORKSPACE_CONFIGURED=false

if [ $# -eq 0 ] ; then NO_ARGS=true ; fi

while getopts "acom" flag ; do
    case $flag in
        a) RUN_ATOMSPACE_BUILDSLAVE=true ;;
        c) RUN_COGUTIL_BUILDSLAVE=true ;;
        o) RUN_OPENCOG_BULDSLAVE=true ;;
        m) RUN_MOSES_BULDSLAVE=true ;;
        h) usage ;;
        \?) usage ;;
        *)  UNKNOWN_FLAGS=true ;;
    esac
done

## AtomSpace
if [ $RUN_ATOMSPACE_BUILDSLAVE ] ; then
    while [ $ATOMSPACE_WORKSPACE_CONFIGURED == false ] ; do
        if [ -a /var/workspace/atomspace/buildbot.tac ]; then
            echo "----atomspace buildslave workspace is configured."
            # the remove is required so as to enable restart when container
            # fails, as twisted.pid is a lock against multiple instances.
            if [ -a /var/workspace/atomspace/twistd.pid ]; then
                rm /var/workspace/atomspace/twistd.pid
                echo "----Removed stale twisted.pid file from atomspace \
                    buildslave workspace."
            fi
            # cogutil is required for tests to run.
            echo "----Installing/Updating cogutil."
            sudo /tmp/octool -c
            echo "----Installed/Updated cogutil."

            # this is set to true so as to avoid an infinit loop should the
            # start of the buildslave fail.
            ATOMSPACE_WORKSPACE_CONFIGURED=true
            sleep 30s # This is to give time for master to finish setting up.
            buildslave start --nodaemon /var/workspace/atomspace
        fi

        echo "----atomspace buildslave workspace not configured yet."
        sleep 10s
    done
fi

## Cogutil
if [ $RUN_COGUTIL_BUILDSLAVE ] ; then
    while [ $COGUTIL_WORKSPACE_CONFIGURED == false ] ; do
        if [ -a /var/workspace/cogutil/buildbot.tac ]; then
            echo "----cogutil buildslave workspace is configured."
            # the remove is required so as to enable restart when container
            # fails, as twisted.pid is a lock against multiple instances.
            if [ -a /var/workspace/cogutil/twistd.pid ]; then
                rm /var/workspace/cogutil/twistd.pid
                echo "----Removed stale twisted.pid file from cogutil \
                    buildslave workspace."
            fi

            # this is set to true so as to avoid an infinit loop should the
            # start of the buildslave fail.
            COGUTIL_WORKSPACE_CONFIGURED=true
            sleep 30s # This is to give time for master to finish setting up.
            buildslave start --nodaemon /var/workspace/cogutil
        fi

        echo "----cogutil buildslave workspace not configured yet."
        sleep 10s
    done
fi

## OpenCog
if [ $RUN_OPENCOG_BULDSLAVE ] ; then
    while [ $OPENCOG_WORKSPACE_CONFIGURED == false ] ; do
        if [ -a /var/workspace/opencog/buildbot.tac ]; then
            echo "----opencog buildslave workspace is configured."
            # the remove is required so as to enable restart when container
            # fails, as twisted.pid is a lock against multiple instances.
            if [ -a /var/workspace/opencog/twistd.pid ]; then
                rm /var/workspace/opencog/twistd.pid
                echo "----Removed stale twisted.pid file from opencog \
                    buildslave workspace."
            fi

            # cogutil and atomspace are required for tests to run.
            echo "----Installing/Updating cogutil and atomspace."
            sudo /tmp/octool -ca
            echo "----Installed/Updated cogutil and atomspace."

            # this is set to true so as to avoid an infinit loop should the
            # start of the buildslave fail.
            OPENCOG_WORKSPACE_CONFIGURED=true
            sleep 30s # This is to give time for master to finish setting up.
            buildslave start --nodaemon /var/workspace/opencog
        fi

        echo "----opencog buildslave workspace not configured yet."
        sleep 10s
    done
fi

## MOSES
if [ $RUN_MOSES_BULDSLAVE ] ; then
    while [ $MOSES_WORKSPACE_CONFIGURED == false ] ; do
        if [ -a /var/workspace/moses/buildbot.tac ]; then
            echo "----moses buildslave workspace is configured."
            # the remove is required so as to enable restart when container
            # fails, as twisted.pid is a lock against multiple instances.
            if [ -a /var/workspace/moses/twistd.pid ]; then
                rm /var/workspace/moses/twistd.pid
                echo "----Removed stale twisted.pid file from moses \
                    buildslave workspace."
            fi

            # cogutil is required for tests to run.
            echo "----Installing/Updating cogutil."
            sudo /tmp/octool -c
            echo "----Installed/Updated cogutil."

            # this is set to true so as to avoid an infinit loop should the
            # start of the buildslave fail.
            MOSES_WORKSPACE_CONFIGURED=true
            sleep 30s # This is to give time for master to finish setting up.
            buildslave start --nodaemon /var/workspace/moses
        fi

        echo "----moses buildslave workspace not configured yet."
        sleep 10s
    done
fi

if [ $UNKNOWN_FLAGS ] ; then usage ; fi
if [ $NO_ARGS ] ; then usage ; fi
