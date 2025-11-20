#
# Container with AtomSpace plus extra python and node.js stuff.
#
# Usage:
# 1. docker build -t opencog/atomspace-py .
# 2. docker create --name atompy  -it opencog/atomspace-py
# 3. docker start -i atompy
#
#
ARG OS_VERSION=latest
ARG BASE_IMAGE=opencog/atomspace:${OS_VERSION}
FROM ${BASE_IMAGE}

# Install Debian-managed packages to get base python abilities.
RUN apt-get -y update && apt-get -y upgrade && apt-get -y autoremove
RUN apt-get -y install python3-pip python3-venv

# Install Debian-managed packages to get base node.js abilities.
RUN apt-get -y install npm

# Everything else happens as the user.
USER opencog
WORKDIR /home/opencog

# Set up NPM repos so that they install as the user.
RUN mkdir -p ~/.npm-global
RUN npm config set prefix ~/.npm-global
RUN echo 'export PATH=~/.npm-global/bin:$PATH' >> ~/.bashrc
ENV PATH=~/.npm-global/bin:$PATH

# Create python gaol located at a popular location
# Activate manually wih `source ~/.venv/bin/activate`
RUN python3 -m venv .venv

# Use uv as a replacement for pipx, poetry, pyenv, virtualenv, twine.
RUN .venv/bin/pip install uv

# Other misc stuff
RUN .venv/bin/pip install requests

# Model Context Protocol MCP python sdk
# This SDK is not available on older OS'es, so just || true so that
# the Docker build does not fail. Of course, its a silent failure, but
# who cares. No one will be using it.
RUN .venv/bin/pip install mcp || true

# Opencog installed itself here; we want it available in the venv.
ENV PYTHONPATH=/usr/local/lib/python3.12/dist-packages:$PYTHONPATH

CMD /bin/bash

ONBUILD USER root
