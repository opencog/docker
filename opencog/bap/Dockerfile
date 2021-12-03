# Step to take
# 1. docker build -t opencog/atomspace:bap .
# 2. docker create --name atomspace-bap -p 17001:17001
#       -v /ABSOLUTE/PATH/TO/YOUR/WORKING/DIRECTORY/On/Your/PC/:/devel
#       -w /devel
#       -it opencog/atomspace:bap
# 3. docker start -i atomspace-bap

FROM opencog/cogutil

RUN apt-get -y install opam

# Switch away from the root user before doing opam work.
USER opencog
WORKDIR /home/opencog

# Inits opam and install the OCaml compiler. Disable sandboxing
# per https://github.com/ocaml/opam-repository/issues/20093
RUN opam init --comp=4.09.0 --disable-sandboxing

# Activate opam environment
RUN eval `opam config env`

RUN opam install -y depext

# Install bap and its dependencies
RUN opam -y depext --install bap -y

RUN eval $(opam env)

# Install core atomspace.
RUN  /tmp/octool -a ; ccache -C

# Docker defaults
CMD /bin/bash

# For images built on this
ONBUILD USER root
