#
# Container with word-pair-counting demo.
#
FROM opencog/learn

# Install a webserver for the visualizer
RUN apt-get -y install apache2 php

USER opencog
WORKDIR /home/opencog

# Install a very simple pair visualizer.
RUN mkdir src && cd src && git clone https://github.com/opencog/cogprotolab.git
RUN sudo cp -pr src/cogprotolab/vis-word-pairs/* /var/www/html/

# Copy user text files into the container.
COPY --chown=opencog:opencog text-pages/* text/input-pages/
RUN rm text/input-pages/README.md

CMD /bin/bash

ONBUILD USER root
