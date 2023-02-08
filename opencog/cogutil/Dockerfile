#
# Steps to take:
#
# 1. docker build --no-cache -t opencog/cogutil
# 2. docker run --rm -it opencog/cogutil

FROM opencog/opencog-deps:latest

# Install cogutil
RUN  /tmp/octool -c && ccache -C

USER opencog

# Docker defaults
CMD bash

# For images built on this
ONBUILD USER root
