# docker build --no-cache -t opencog/cogserver .

FROM opencog/opencog-deps

# Clone opencog and build it
RUN git clone --depth 1 https://github.com/opencog/opencog oc
RUN cd oc; mkdir build; cd build; cmake ..; make -j$(nproc)

# Docker defaults
WORKDIR /home/opencog/oc/build
USER opencog

## Start cogserver when container runs
CMD opencog/server/cogserver
