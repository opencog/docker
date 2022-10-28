#
# Container with core learning repos. No data.
#
# Steps:
# 1. docker build --no-cache -t opencog/learn .
# 2. docker create --name learn -p 17001:17001
#       -it opencog/learn
# Note: the -p flag is external_port:internal_port
# 3. docker start -i learn
#

FROM opencog/atomspace

RUN  /tmp/octool -z && ccache -C

USER opencog
WORKDIR /home/opencog

CMD /bin/bash

ONBUILD USER root
