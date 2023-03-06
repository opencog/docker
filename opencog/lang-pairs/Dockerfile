#
# Container with word-pair-counting demo.
#
FROM opencog/learn:latest

# Install a webserver for the visualizer
RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install apache2 php

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
RUN chmod ugo+x ./count-pairs.sh

CMD /bin/bash

ONBUILD USER root
