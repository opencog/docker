#
# Container with just the core atomspace installed.
#
# Steps:
# 1. docker build -t opencog/atomspace .
# 2. docker create --name atomspace -p 17001:17001
#       -v /ABSOLUTE/PATH/TO/YOUR/WORKING/DIRECTORY/On/Your/PC/:/opencog
#       -w /opencog
#       -it opencog/atomspace
# 3. docker start -i atomspace
#
# Check http://wiki.opencog.org/w/Building_OpenCog on how to use IDEs

FROM opencog/cogutil

# Install core atomspace.
RUN  /tmp/octool -a ; ccache -C

USER opencog
WORKDIR /home/opencog

# Setup the workspace for developers
RUN sudo chown -R opencog:opencog .
RUN sudo chown -R opencog:opencog .*

# XXX FIXME - despite efforts, the timestamp on
#    /usr/local/lib/guile/3.0/site-ccache/opencog.go
# is older than that on
#    /usr/local/share/guile/site/3.0/opencog.scm
# So hack around this by forcing a compile.
RUN guile -c "(use-modules (opencog))"

CMD /bin/bash

ONBUILD USER root
