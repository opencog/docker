# For creating images with non gui tools for development and dependencies installed
# Steps:
# 1. docker build -t opencog/opencog-dev:cli .
# 2. docker create --name opencog -p 17001:17001 -p 5000:5000
#       -v ABSOLUTE/PATH/TO/OPENCOG/ROOT/DIRECTORY:/home/opencog
#       -it opencog/opencog-dev:cli
# 3. docker start -i opencog
# 4. mkdir build
# 5. cmake ..
# 6. make -j$(nproc)
# Check http://wiki.opencog.org/w/Building_OpenCog on how to use IDEs

FROM opencog/opencog-deps
MAINTAINER David Hart "dhart@opencog.org"

ADD odbc.ini /scripts/odbc.ini
ADD configure-database.sh /scripts/configure-database.sh

RUN apt-get install \
                    byobu \
                    less \
                    curl \
                    mosh \
                    vim \
                    pv \
                    bc \
                    gdb \
                    python2.7-dbg\
                    tmux \
                    rlwrap \
                    man
    --assume-yes

USER opencog
COPY /scripts/.gdbinit ~/.gdbinit
CMD /bin/bash
