#
# Container with word-pair-counting demo.
#
FROM opencog/learn:latest

USER opencog
WORKDIR /home/opencog

# Set up the basic working directories
RUN ./setup-for-experiments.sh

# Copy user text files into the container.
COPY --chown=opencog:opencog text-pages/* text/input-pages/
RUN rm text/input-pages/README.md

# Set up and run the fully-auomated pair-counter.
COPY --chown=opencog:opencog /scripts/ .
RUN chmod ugo+x ./count-pairs.sh

# RUN ./count-pairs.sh

CMD /bin/bash

ONBUILD USER root
