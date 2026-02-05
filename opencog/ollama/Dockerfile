#
# Container with AtomSpace plus Ollama
#
# Usage:
# 1. docker build -t opencog/ollama .
# 2. docker create --name aol --hostname aol -it opencog/ollama
# 3. docker start -i aol
#
ARG OS_VERSION=latest
ARG BASE_IMAGE=opencog/atomspace:${OS_VERSION}

FROM ollama/ollama:latest AS ollama

FROM ${BASE_IMAGE}

# Copy ollama binary and libs
COPY --from=ollama /bin/ollama /usr/local/bin/ollama
COPY --from=ollama /usr/lib/ollama /usr/lib/ollama

# Set library path for GPU backends
ENV LD_LIBRARY_PATH=/usr/lib/ollama:$LD_LIBRARY_PATH

# Do everything else as the opencog user.
USER opencog
WORKDIR /home/opencog

# Pull model. We're gonna try qwen3 to start.
RUN ollama serve & \
    until ollama list > /dev/null 2>&1; do sleep 1; done && \
    ollama pull qwen3:8b && \
    kill %1

EXPOSE 11434

# Run tini as PID1 and bash as PID2. tini will manage signals, etc.
ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["/bin/bash"]

ONBUILD USER root
