# Usage: Follow the steps in ../../README.md.

FROM opencog/opencog-deps
MAINTAINER David Hart "dhart@opencog.org"

RUN pip install buildbot-slave
COPY run.sh /home/opencog/
RUN chown opencog:opencog run.sh

USER opencog
WORKDIR /home/opencog

CMD bash
