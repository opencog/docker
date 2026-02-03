#
# Container with AtomSpace plus Anthropic Claude Code
#
# Usage:
# 1. docker build -t opencog/claude-code .
# 2. docker create --name aclaw  -it opencog/claude-code
# 3. docker start -i aclaw
#
FROM opencog/atomspace-py:latest

# Everything else happens as the user.
USER opencog
WORKDIR /home/opencog

# Install Anthropic Claude Code
# RUN npm install -g @anthropic-ai/claude-code
RUN curl -fsSL https://claude.ai/install.sh | bash

# The OAuth token needs to be refreshed from time to time.
RUN curl -O https://raw.githubusercontent.com/RavenStorm-bit/claude-token-refresh/main/claude_token_refresh.py && chmod +x claude_token_refresh.py

# Run tini as PID1 and bash as PID2. tini will manage signals, etc.
ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["/bin/bash"]

ONBUILD USER root
