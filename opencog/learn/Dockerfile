#
# Container with core learning repos. No data.
#
FROM opencog/atomspace

RUN  /tmp/octool -z && ccache -C

USER opencog
WORKDIR /home/opencog
CMD /bin/bash

ONBUILD USER root
