#
# Container with core learning repos. No data.
#
FROM opencog/atomspace:latest
# FROM opencog/atomspace:ubuntu-20.04
# FROM opencog/atomspace:ubuntu-22.04
# FROM opencog/atomspace:ubuntu-24.04
# FROM opencog/atomspace:ubuntu-25.10
# FROM opencog/atomspace:debian-bullseye
# FROM opencog/atomspace:debian-bookworm
# FROM opencog/atomspace:debian-trixie

# Install everything learn-related.
RUN  /tmp/octool -z && ccache -C

# Uncomment to instal GPU stuff
# RUN  /tmp/octool -u && ccache -C

# uncomment to install vision stuff
# RUN  /tmp/octool -n && ccache -C

USER opencog
WORKDIR /home/opencog

# Compile the scheme files as the user.
RUN guile -c "(use-modules (opencog matrix) (opencog nlp) (opencog learn))"

# Place a basic recommended configuration where the user can find it.
COPY /scripts/ /home/opencog
RUN sudo chown -R opencog:opencog setup-for-experiments.sh && \
	chmod ugo+x setup-for-experiments.sh

CMD /bin/bash

ONBUILD USER root
