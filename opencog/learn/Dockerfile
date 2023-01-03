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
# 4. cd /home/opencog/experiments/run-1
# 5. Review the config files; change as desired.
# 6. Run run-tmux.sh

FROM opencog/atomspace

RUN  /tmp/octool -z && ccache -C

USER opencog
WORKDIR /home/opencog

RUN mkdir data
RUN mkdir text
RUN mkdir text/input-pages
COPY --chown=opencog:opencog text-pages/* text/input-pages/
RUN rm text/input-pages/README.md


RUN mkdir experiments
RUN mkdir experiments/run-1
RUN cp -r /usr/local/share/opencog/learn/run-config/*.sh experiments/run-1
RUN cp -r /usr/local/share/opencog/learn/run experiments/run-1

CMD /bin/bash

ONBUILD USER root
