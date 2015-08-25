# Usage: Follow the steps in ../../README.md.

FROM opencog/opencog-deps
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get install -y doxygen graphviz; pip install buildbot-slave
COPY run.sh /home/opencog/
RUN chown opencog:opencog run.sh

USER opencog
WORKDIR /home/opencog

# For Haskell
RUN /tmp/octool -s

CMD bash
