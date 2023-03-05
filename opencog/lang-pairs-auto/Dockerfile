#
# Container with word-pair-counting demo.
#
FROM opencog/learn:latest

USER opencog
WORKDIR /home/opencog

# Copy user text files into the container.
COPY --chown=opencog:opencog text-pages/* text/input-pages/
RUN rm text/input-pages/README.md

CMD /bin/bash

ONBUILD USER root
