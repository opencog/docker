# Container with just the core atomspace installed.
# Steps:
# 1. docker build -t opencog/atomspace .
# 2. docker create --name atomspace -p 17001:17001
#       -v /ABSOLUTE/PATH/TO/OPENCOG/ROOT/DIRECTORY/On/Your/PC/:/opencog
#       -w /opencog
#       -it opencog/atomspace
# 3. docker start -i atomspace
# 4. mkdir build
# 5. cmake ..
# 6. make -j$(nproc)
# Check http://wiki.opencog.org/w/Building_OpenCog on how to use IDEs

FROM opencog/cogutil

# Install core atomspace.
RUN  /tmp/octool -a ; ccache -C

USER opencog
WORKDIR /home/opencog

# Setup the workspace for developers
RUN sudo chown -R opencog:opencog .*

CMD /bin/bash

ONBUILD USER root
