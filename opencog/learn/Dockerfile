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

# Set up a reasonable user environment.
RUN mkdir text && mkdir text/input-pages
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
