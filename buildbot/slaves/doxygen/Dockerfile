# Usage: Follow the steps in ../../README.md.

FROM opencog/opencog-deps

RUN apt-get install -y doxygen graphviz; pip install buildbot-slave ; \
     /tmp/setup.sh -c
COPY run.sh /home/opencog/
RUN chown opencog:opencog run.sh

USER opencog
CMD bash
