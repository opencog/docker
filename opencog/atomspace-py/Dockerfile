#
# Container with AtomSpace plus extra python and node.js stuff.
#
# Usage:
# 1. docker build -t opencog/atomspace-py .
# 2. docker create --name atompy  -it opencog/atomspace-py
# 3. docker start -i atompy
#
#
FROM opencog/atomspace:latest

# Install Debian-managed packages to get base python abilities.
RUN apt-get -y install python3-pip python3-venv

# Install Debian-managed packages to get base node.js abilities.
RUN apt-get -y install npm

# Everything else happens as the user.
USER opencog
WORKDIR /home/opencog

# Create python gaol located at a popular location
RUN python3 -m venv .venv

# Use uv as a replacement for pipx, poetry, pyenv, virtualenv, twine.
RUN .venv/bin/pip install uv

CMD /bin/bash

ONBUILD USER root
