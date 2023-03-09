#
# Container for word-pair counting; both demo and fully-automated.
#
FROM opencog/learn:latest

USER opencog
WORKDIR /home/opencog

# Install a very simple pair visualizer.
RUN mkdir src && cd src && git clone https://github.com/opencog/cogprotolab.git
RUN sudo chown opencog:opencog /var/www/html/
RUN sudo cp -pr src/cogprotolab/vis-word-pairs/* /var/www/html/

# Set up the basic working directories
RUN ./setup-for-experiments.sh

# Copy in the fully-automated pair-counter.
COPY --chown=opencog:opencog /scripts/ .
COPY --chown=opencog:opencog /demo-dict-pair/ demo-dict-pair

CMD /bin/bash

ONBUILD USER root
