#
# Container with AtomSpace plus Anthropic Claude
#
# Usage:
# 1. docker build -t opencog/atomspace-claude .
# 2. docker create --name atomclaude  -it opencog/atomspace-claude
# 3. docker start -i atomclaude
#
FROM opencog/atomspace-py:latest

# Install Anthropic Claude Code
RUN npm install -g @anthropic-ai/claude-code

# Everything else happens as the user.
USER opencog
WORKDIR /home/opencog

CMD /bin/bash

ONBUILD USER root
