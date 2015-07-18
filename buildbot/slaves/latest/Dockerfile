# Mount the relevant host directory to /home/buildslave/slave
# This is based on the default working environment for opencog
#
# Steps:
# 1. docker build --no-cache -t opencog/buildslave:zera .
# 2. docker stop zera
# 3. docker rm zera
# 4. docker run -d --name zera --link buildmaster:zera-master \
#    -v $PWD:/home/buildslave/slave \
#    -it opencog/buildslave:zera

FROM opencog/opencog-deps
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get update
RUN pip install buildbot-slave

# Create and switch user. This was added for security
# Disbaled temporarily for expediancy
# RUN adduser --disabled-password --gecos "Continuese Integration" buildslave
# USER buildslave
WORKDIR /home/buildslave

# Defaults
# This assumes the  $RELEVANT_DIRECTORY_PATH has the necessary files created on
# running `buildslave create-slave slave zera-master zera XXXXXX`
CMD buildslave start  --nodaemon slave
