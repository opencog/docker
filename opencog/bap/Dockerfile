# Step to take
# 1. docker build -t opencog/atomspace:bap .
# 2. docker create --name atomspace:bap -p 17001:17001
#       -v /ABSOLUTE/PATH/TO/YOUR/WORKING/DIRECTORY/On/Your/PC/:/devel
#       -w /devel
#       -it opencog/atomspace:bap
# 3. docker start -it opencog/atomspace:bap

FROM opencog/atomspace

RUN apt-get -y install opam

# Inits opam and install the OCaml compiler. Disable sandboxing
# per https://github.com/ocaml/opam-repository/issues/20093
RUN opam init --comp=4.09.0 --disable-sandboxing

# Activate opam environment
RUN eval `opam config env`

RUN opam install -y depext

# Install bap and its dependencies
RUN opam -y depext --install bap -y

RUN eval $(opam env)

USER opencog

# Docker defaults
CMD bash

# For images built on this
ONBUILD USER root
