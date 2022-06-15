#
# Container with crude language data.
#
# Steps:
# 1. docker build -t opencog/lang-model .
# 2. docker create --name lang-model -p 17001:17001
#       -it opencog/lang-model
# 3. docker start -i lang-model
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

# Setup the workspace for developers
# RUN sudo chown -R opencog:opencog .
# RUN sudo chown -R opencog:opencog .*

WORKDIR /home/opencog
CMD /bin/bash

ONBUILD USER root
