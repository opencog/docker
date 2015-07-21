#!/bin/bash
#
# This script is used for running the doxygen builder.
DOXYGEN_WORKSPACE_CONFIGURED=false

while [ $DOXYGEN_WORKSPACE_CONFIGURED == false ] ; do
    if [ -a /var/workspace/slaves/doxygen/buildbot.tac ]; then
        echo "----doxygen buildslave workspace is configured."
        WORKSPACE_CONFIGURED=true
        sleep 30s # This is to give time for master to finish setting up.
        buildslave start --nodaemon /var/workspace/slaves/doxygen
    fi
    echo "----doxygen buildslave workspace not configured yet."
    sleep 10s
done
