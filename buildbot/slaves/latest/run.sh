#!/bin/bash
#
# This script is used for running the builders. Pass only a single

# Functions
usage() {
printf "Usage: ./$SELF_NAME [OPTIONS]

  OPTIONS:
    -a Wait for configuration of workspace and run buildslave process for
       atomspace tests.
    -c Wait for configuration of workspace and run buildslave process for
       cogutils tests.
    -o Wait for configuration of workspace and run buildslave process for
       cogutils tests.
    -h This help message. \n"
}


# Main Execution
ATOMSPACE_WORKSPACE_CONFIGURED=false
COGUTILS_WORKSPACE_CONFIGURED=false
OPENCOG_WORKSPACE_CONFIGURED=false

if [ $# -eq 0 ] ; then NO_ARGS=true ; fi

while getopts "aco" flag ; do
    case $flag in
        a) RUN_ATOMSPACE_BUILDSLAVE=true ;;
        c) RUN_COGUTILS_BUILDSLAVE=true ;;
        o) RUN_OPENCOG_BULDSLAVE=true ;;
        h) usage ;;
        \?) usage ;;
        *)  UNKNOWN_FLAGS=true ;;
    esac
done

## AtomSpace
if [ $RUN_ATOMSPACE_BUILDSLAVE ] ; then
    while [ $ATOMSPACE_WORKSPACE_CONFIGURED == false ] ; do
        if [ -a /var/workspace/slaves/atomspace/buildbot.tac ]; then
            echo "----atomspace buildslave workspace is configured."
            # the remove is required so as to enable restart when container
            # fails, as twisted.pid is a lock against multiple instances.
            if [ -a /var/workspace/slaves/atomspace/twistd.pid ]; then
                rm /var/workspace/slaves/atomspace/twistd.pid
                echo "----Removed stale twisted.pid file from atomspace \
                    buildslave workspace."
            fi
            # cogutils is required for tests to run.
            echo "----Installing/Updating cogutils."
            sudo /tmp/setup.sh -c
            echo "----Installed/Updated cogutils."

            # this is set to true so as to avoid an infinit loop should the
            # start of the buildslave fail.
            ATOMSPACE_WORKSPACE_CONFIGURED=true
            sleep 30s # This is to give time for master to finish setting up.
            buildslave start --nodaemon /var/workspace/slaves/atomspace
        fi

        echo "----atomspace buildslave workspace not configured yet."
        sleep 10s
    done
fi

## Cogutils
if [ $RUN_COGUTILS_BUILDSLAVE ] ; then
    while [ $COGUTILS_WORKSPACE_CONFIGURED == false ] ; do
        if [ -a /var/workspace/slaves/cogutils/buildbot.tac ]; then
            echo "----cogutils buildslave workspace is configured."
            # the remove is required so as to enable restart when container
            # fails, as twisted.pid is a lock against multiple instances.
            if [ -a /var/workspace/slaves/cogutils/twistd.pid ]; then
                rm /var/workspace/slaves/cogutils/twistd.pid
                echo "----Removed stale twisted.pid file from cogutils \
                    buildslave workspace."
            fi

            # this is set to true so as to avoid an infinit loop should the
            # start of the buildslave fail.
            COGUTILS_WORKSPACE_CONFIGURED=true
            sleep 30s # This is to give time for master to finish setting up.
            buildslave start --nodaemon /var/workspace/slaves/cogutils
        fi

        echo "----cogutils buildslave workspace not configured yet."
        sleep 10s
    done
fi

## OpenCog
if [ $RUN_OPENCOG_BULDSLAVE ] ; then
    while [ $OPENCOG_WORKSPACE_CONFIGURED == false ] ; do
        if [ -a /var/workspace/slaves/opencog/buildbot.tac ]; then
            echo "----opencog buildslave workspace is configured."
            # the remove is required so as to enable restart when container
            # fails, as twisted.pid is a lock against multiple instances.
            if [ -a /var/workspace/slaves/opencog/twistd.pid ]; then
                rm /var/workspace/slaves/opencog/twistd.pid
                echo "----Removed stale twisted.pid file from opencog \
                    buildslave workspace."
            fi

            # cogutils and atomspace are required for tests to run.
            echo "----Installing/Updating cogutils and atomspace."
            sudo /tmp/setup.sh -ca
            echo "----Installed/Updated cogutils and atomspace."

            # this is set to true so as to avoid an infinit loop should the
            # start of the buildslave fail.
            OPENCOG_WORKSPACE_CONFIGURED=true
            sleep 30s # This is to give time for master to finish setting up.
            buildslave start --nodaemon /var/workspace/slaves/opencog
        fi

        echo "----opencog buildslave workspace not configured yet."
        sleep 10s
    done
fi

if [ $UNKNOWN_FLAGS ] ; then usage ; fi
if [ $NO_ARGS ] ; then usage ; fi
