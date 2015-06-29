# Step to take
# 1. docker build --no-cache -t opencog/moses .
# 2. docker run -v $PWD:/home/moses -it opencog/moses bash
# 3. moses

# Note:
# 1. Since the build on r-base offical image R is installed on the system
# 1. R is installed because there is an r binding being developed
#    @ https://github.com/mjsduncan/Rmoses . Since there is no cran
#    package for it devtools should be used. An Rscript has to be written (I guess)
# 2. Step 2 above assumes you are running the command from a directory
# containing some data. Instead of $PWD you could use abslute path like
# $HOME/path/to/data/

FROM ubuntu:14.04
RUN apt-get update ; apt-get -y upgrade ; apt-get -y autoclean

# Install dependencies
RUN apt-get install -y libboost-date-time-dev \
                       libboost-filesystem-dev \
                       libboost-program-options-dev \
                       libboost-regex-dev \
                       libboost-thread-dev \
                       libboost-system-dev \
                       libboost-math-dev \
                       libboost-mpi-dev

# Install Utilities & R
RUN apt-get -y install build-essential cmake sudo git wget r-base

# Build and Install moses
WORKDIR /tmp/
ADD https://raw.githubusercontent.com/opencog/opencog/master/scripts/ocpkg \
    /tmp/setup.sh
RUN chmod +x /tmp/setup.sh &&  /tmp/setup.sh -cm

# Create and switch user. The user is privileged, with no password required.
# That is you can use sudo.
RUN adduser --disabled-password --gecos "MOSES USER" moses
RUN adduser moses sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Docker Defaults
USER moses
WORKDIR /home/moses
CMD bash
