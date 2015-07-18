# using opencog's base image b/c one might want compile
FROM busybox

# Configure Workspace used by buildbot master & slaves
RUN mkdir -p /var/workspace/master /var/workspace/slaves ; \
    chown default:default /var/workspace

VOLUME /var/workspace
