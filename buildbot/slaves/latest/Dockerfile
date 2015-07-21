# Usage: Follow the steps in ../../README.md.

FROM opencog/opencog-deps
MAINTAINER David Hart "dhart@opencog.org"

RUN pip install buildbot-slave
COPY run.sh /home/opencog/

# The script is required for installing and updating buildslave containers.
ADD https://raw.githubusercontent.com/opencog/opencog/master/scripts/ocpkg \
    /tmp/setup.sh
RUN chmod u+x /tmp/setup.sh ; chown opencog:opencog run.sh /tmp/setup.sh

USER opencog
WORKDIR /home/opencog

CMD bash
