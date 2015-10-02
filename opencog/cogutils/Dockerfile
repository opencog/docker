# Step to take
# 1. docker build --no-cache -t opencog/cogutils
# 2. docker run --rm -it opencog/cogutils

FROM opencog/opencog-deps

# Install cogutils
RUN  /tmp/octool -c

USER opencog

# Docker defaults
# CMD bash for maintenance only
# ENTRYPOINT should be an application like distcc or buildbot or cogserver
CMD bash

# For images built on this
ONBUILD USER root
