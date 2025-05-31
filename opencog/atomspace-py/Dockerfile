#
# Container with AtomSpace plus lots of extra python stuff.
#
FROM opencog/atomspace:latest

# First, install Debian-managed packages.
RUN apt-get -y install python3-pip python3-venv

# Everything else happens as the user.
USER opencog
WORKDIR /home/opencog

# Create python gaol located at a popular location
RUN python3 -m venv .venv

# Use uv as a replacement for pipx, poetry, pyenv, virtualenv, twine.
RUN .venv/bin/pip install uv

CMD /bin/bash

ONBUILD USER root
