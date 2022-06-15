#
# Container with crude language data.
# Experimental. This does NOT do anything interesting! Demo only.
#
# Steps:
# 1. docker build -t opencog/lang-model .
# 2. docker create --name lang-model -p 17001:19014
#       -it opencog/lang-model
# Note: the -p flag is external_port:internal_port
# 3. docker start -i lang-model
# 4. cd experiments/run-word-pairs
# 5. guile -l scm/cogserver-nav.scm
#

FROM opencog/atomspace

USER opencog
WORKDIR /home/opencog

RUN mkdir /home/opencog/data
WORKDIR /home/opencog/data

# RUN wget --progress=bar:force:noscroll https://linas.org/datasets/r13-all-in-one.tar.bz2
RUN curl https://linas.org/datasets/r13-all-in-one.tar.bz2 -o r13-all-in-one.tar.bz2
RUN tar -jxf r13-all-in-one.tar.bz2
RUN rm r13-all-in-one.tar.bz2

COPY /scripts/ /home/opencog

# Setup the workspace for developers
WORKDIR /home/opencog
RUN sudo chown -R opencog:opencog .[a-z]*
RUN sudo chown -R opencog:opencog experiments

CMD /bin/bash

ONBUILD USER root
