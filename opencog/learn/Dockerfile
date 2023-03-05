#
# Container with core learning repos. No data.
#
FROM opencog/atomspace:latest

# Install everything learn-related.
RUN  /tmp/octool -z && ccache -C

USER opencog
WORKDIR /home/opencog

# Compile the scheme files as the user.
RUN guile -c "(use-modules (opencog nlp))"
RUN guile -c "(use-modules (opencog learn))"

# Place a basic recommended configurtion where the user can find it.
COPY /scripts/ /home/opencog
RUN sudo chown -R opencog:opencog setup-for-experiments.sh
RUN chmod ugo+x setup-for-experiments.sh

CMD /bin/bash

ONBUILD USER root
