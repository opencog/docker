#
# Container with core learning repos. No data.
#
# Steps:
# 0. Copy your text files to `text-files`. These will then be
#    automatically copied into the Docker container, to the
#    directory `text/input-files`.
# 1. docker build --no-cache -t opencog/learn .
# 2. docker create --name learn -p 17001:17001 -it opencog/learn
#    Note: the -p flag is external_port:internal_port
# 3. docker start -i learn
# 4. cd experiments/run-1
# 5. Review the config files; change as desired.
# 6. . 0-pipeline.sh  # i.e. source the contents of this config file.
# 7. run/run-tmux.sh  # Set up multiple byobu terminals.
# 8. Review the project README's and follow those ...
#
# For example:
# A. Go to the `cogsrv` tab, and run `run/2-word-pairs/run-cogserver.sh`
# B. Go to the `telnet` tab, and run `rlwrap telnet localhost 17001`
# C. Go to the `submit` tab, and run `run/2-word-pairs/pair-submit.sh`
# D. Wait until the telnet stats clear ...

FROM opencog/atomspace

RUN  /tmp/octool -z && ccache -C

USER opencog
WORKDIR /home/opencog

# Copy user text files into the container.
RUN mkdir text
RUN mkdir text/input-pages
COPY --chown=opencog:opencog text-pages/* text/input-pages/
RUN rm text/input-pages/README.md

# Set up a reasonable user environment.
RUN mkdir data
RUN mkdir experiments
RUN mkdir experiments/run-1
RUN cp -r /usr/local/share/opencog/learn/run-config/*.sh experiments/run-1
RUN cp -r /usr/local/share/opencog/learn/run-common experiments/run-common
RUN chmod ugo+x experiments/run-common/*.sh
RUN chmod ugo+x experiments/run-common/*.pl
RUN cp -r /usr/local/share/opencog/learn/run experiments/run-1
RUN chmod ugo+x experiments/run-1/run/*.sh
RUN chmod ugo+x experiments/run-1/run/*/*.sh

CMD /bin/bash

ONBUILD USER root
