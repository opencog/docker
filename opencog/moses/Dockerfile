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

FROM opencog/cogutil

# Install R
RUN apt-get -y install r-base

# Install moses
RUN /tmp/octool -m && ccache -C

# Docker Defaults
USER opencog
CMD bash
