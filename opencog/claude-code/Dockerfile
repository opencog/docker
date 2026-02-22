#
# Container with AtomSpace plus Anthropic Claude Code
#
# Usage:
# 1. docker build -t opencog/claude-code .
# 2. docker create --name aclaw --hostname aclaw
#       -p 17001:17001 -p 18080:18080 -p 18081:18081
#       -v /home/linas/src-aclaw/:/home/opencog/src
#       -w /home/opencog/
#       -it opencog/claude-code
# 3. docker start -i aclaw
#
# In the container:
# 4. ./run-tmux.sh
# 5. cd src
# 6. /home/opencog/.local/bin/claude

#
ARG OS_VERSION=latest
# We don't actually need npm/python any more.
# ARG BASE_IMAGE=opencog/atomspace-py:${OS_VERSION}
ARG BASE_IMAGE=opencog/atomspace:${OS_VERSION}
FROM ${BASE_IMAGE}

# Everything else happens as the user.
USER opencog
WORKDIR /home/opencog

# Install Anthropic Claude Code
# RUN npm install -g @anthropic-ai/claude-code
RUN curl -fsSL https://claude.ai/install.sh | bash
ENV PATH=/home/opencog/.local/bin:$PATH

# The OAuth token needs to be refreshed from time to time.
# XXX Is this actually needed any more ??
RUN curl -O https://raw.githubusercontent.com/RavenStorm-bit/claude-token-refresh/main/claude_token_refresh.py && chmod +x claude_token_refresh.py

# Run tini as PID1 and bash as PID2. tini will manage signals, etc.
ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["/bin/bash"]

ONBUILD USER root
