# Step to take
# 1. docker build --no-cache -t opencog/moses .
# 2. docker run -v $PWD:/home/moses -it opencog/moses bash
# 3. moses

# Note:
# 1. Since the build on r-base offical image R is installed on the system
# 2. The base image was choosen to be r-base because there is an r binding being
#    developed @ https://github.com/mjsduncan/Rmoses . Since there is no cran
#    package for it devtools should be used. An Rscript has to be written (I guess)
# 3. Step 2 above assumes you are running the command from a directory containing
#    some data. Instead of $PWD you could use abslute path $HOME/path/to/data/

FROM r-base

RUN apt-get update
RUN apt-get -y upgrade

# Install dependencies
RUN apt-get install -y libboost-date-time-dev \
                       libboost-filesystem-dev \
                       libboost-program-options-dev \
                       libboost-regex-dev \
                       libboost-thread-dev \
                       libboost-system-dev \
                       libboost-math-dev \
                       libboost-mpi-dev

# Install Utilities
RUN apt-get -y install git build-essential cmake sudo

# Build and Install moses
WORKDIR /tmp/
## A full clone is required because OC2MOSES.sh checks for an old tag.
RUN git clone https://github.com/opencog/opencog.git
RUN ./opencog/scripts/learning/moses/extract_moses/OC2MOSES.sh moses
RUN mkdir moses/build ; cd moses/build ; cmake .. ; \
    make -j$(nproc) ; make install ; ldconfig

# Create and switch user. The user is privileged, with no password required.
# That is you can use sudo.
RUN adduser --disabled-password --gecos "MOSES USER" moses
RUN adduser moses sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Docker Defaults
USER moses
WORKDIR /home/moses
CMD bash

