#!/bin/bash
#
# This script is used for running the builders.
#/var/workspace/slaves/zera/buildbot.tac
WORKSPACE_CONFIGURED=false

while [ $WORKSPACE_CONFIGURED == false ] ; do
    if [ -a /var/workspace/slaves/zera/buildbot.tac ]; then
        echo "----zera's workspace is configured"
        WORKSPACE_CONFIGURED=true
        sleep 30s # This is to give time for master to finish setting up.
        buildslave start --nodaemon /var/workspace/slaves/zera
    fi
    echo "----zera's workspace not configured yet."
    sleep 10s
done
