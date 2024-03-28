#
# Container with Maximum-Spanning-Tree demo.
#
FROM opencog/learn:latest

USER opencog
WORKDIR /home/opencog

# Set up the basic working directories
RUN ./setup-for-experiments.sh

# Copy in the fully-automated MST scripts.
COPY --chown=opencog:opencog /scripts/ .

# Copy in the demo Link Grammar dictionary.
COPY --chown=opencog:opencog /demo-dict-mpg/ demo-dict-mpg

CMD /bin/bash

ONBUILD USER root
