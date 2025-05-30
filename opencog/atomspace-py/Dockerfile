#
# Container with AtomSpace plus lots of extra python stuff.
#
FROM opencog/atomspace:latest

# First, install Debian-manaaged packages, invlucing pipx for venv.
RUN apt-get -y install python3-pip pipx

USER opencog
WORKDIR /home/opencog

CMD /bin/bash

ONBUILD USER root
