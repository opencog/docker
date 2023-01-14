#
# Container with core learning repos. No data.
#
FROM opencog/atomspace

# Install everythign learn-related.
RUN  /tmp/octool -z && ccache -C

# Compile the scheme files now. octool did not do this.
RUN guile -c "(use-modules (opencog nlp))"
RUN guile -c "(use-modules (opencog learn))"

USER opencog
WORKDIR /home/opencog
CMD /bin/bash

ONBUILD USER root
