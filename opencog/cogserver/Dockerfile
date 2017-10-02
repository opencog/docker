# Step to take
# 1. docker build --no-cache -t opencog/cogserver .
# 2. docker run -p 17001:17001 -it opencog/cogserver
# 3. rlwrap telnet localhost 17001

FROM opencog/cogutil

# Install non-system project dependencies
RUN  /tmp/octool -al ; ccache -C

# Clone opencog and build it
RUN git clone --depth 1 https://github.com/opencog/opencog oc
RUN cd oc; mkdir build; cd build; cmake ..; make -j$(nproc)

# Docker defaults
WORKDIR /home/opencog/oc/build
USER opencog

## Start cogserver when container runs
CMD opencog/server/cogserver
