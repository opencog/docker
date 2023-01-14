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
RUN sudo cp -pr src/cogprotolab/vis-word-pairs/ /var/www/html/

# Copy user text files into the container.
RUN mkdir text && mkdir text/input-pages
COPY --chown=opencog:opencog text-pages/* text/input-pages/
RUN rm text/input-pages/README.md

# Set up a reasonable user environment.
RUN mkdir data
RUN mkdir experiments && mkdir experiments/run-1
RUN cp -r /usr/local/share/opencog/learn/run-config/*.sh experiments/run-1
RUN cp -r /usr/local/share/opencog/learn/run-common experiments/run-common
RUN chmod ugo+x experiments/run-common/*.sh
RUN chmod ugo+x experiments/run-common/*.pl
RUN cp -r /usr/local/share/opencog/learn/run experiments/run-1
RUN chmod ugo+x experiments/run-1/run/*.sh
RUN chmod ugo+x experiments/run-1/run/*/*.sh

CMD /bin/bash

ONBUILD USER root
