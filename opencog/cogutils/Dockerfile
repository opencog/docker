# Step to take
# 1. docker build --no-cache -t opencog/cogutils
# 2. docker run --rm -it opencog/cogutils

FROM opencog/opencog-deps

# Install repositories and dependencies
# FIXME: when octools auto-updates remove this.
ADD https://raw.githubusercontent.com/opencog/ocpkg/master/ocpkg \
    /tmp/octool
RUN chmod 755 /tmp/octool

# Install cogutils
RUN  /tmp/octool -c ; ccache -C

USER opencog

# Docker defaults
CMD bash

# For images built on this
ONBUILD USER root
