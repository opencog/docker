#
# Container with AtomSpace plus Anthropic Claude
#
# Usage:
# 1. docker build -t opencog/atomspace-claude .
# 2. docker create --name atomclaw  -it opencog/atomspace-claude
# 3. docker start -i atomclaw
#
FROM opencog/atomspace-py:latest

# Everything else happens as the user.
USER opencog
WORKDIR /home/opencog

# Install Anthropic Claude Code
RUN npm install -g @anthropic-ai/claude-code

CMD /bin/bash

ONBUILD USER root
