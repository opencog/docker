# For creating images with non gui tools for development and dependencies installed
# Steps:
# 1. docker build -t opencog/opencog-dev:cli .
# 2. docker create --name opencog -p 17001:17001 -p 5000:5000
#       -v /ABSOLUTE/PATH/TO/OPENCOG/ROOT/DIRECTORY/On/Your/PC/:/opencog
#       -w /opencog
#       -it opencog/opencog-dev:cli
# 3. docker start -i opencog
# 4. mkdir build
# 5. cmake ..
# 6. make -j$(nproc)
# Check http://wiki.opencog.org/w/Building_OpenCog on how to use IDEs

FROM opencog/atomspace:latest

# Download data and install opencog.
RUN  /tmp/octool -ow && ccache -C

# Install tools for developers.
RUN apt-get -y install python3-dbg man valgrind gdb

USER opencog
WORKDIR /home/opencog

# Setup the workspace for developers
## Dont't mount host volumes to opencog's home directory inside the container,
## if you do you will not find the contents copied below
COPY /scripts/ /home/opencog
RUN sudo chown -R opencog:opencog .[a-z]*

CMD /bin/bash

ONBUILD USER root
