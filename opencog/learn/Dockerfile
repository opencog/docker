#
# Container with core learning repos. No data.
#
FROM opencog/atomspace

# Install everythign learn-related.
RUN  /tmp/octool -z && ccache -C

USER opencog
WORKDIR /home/opencog

# Compile the scheme files as the user.
RUN guile -c "(use-modules (opencog nlp))"
RUN guile -c "(use-modules (opencog learn))"

CMD /bin/bash

ONBUILD USER root
