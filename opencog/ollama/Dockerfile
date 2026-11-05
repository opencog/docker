#
# Container with AtomSpace plus Ollama
#
# Usage:
# 1. docker build -t opencog/ollama .
# 2. docker create --name aol --hostname aol --gpus all \
#        -p 18082:18082 -p 18083:18083 \
#        -p 11434:11434 -e OLLAMA_HOST=0.0.0.0 \
#        -v /home/linas/foobar:/home/opencog/foobar \
#        -it opencog/ollama
# 3. docker start -i aol
#
ARG OS_VERSION=latest
ARG BASE_IMAGE=opencog/atomspace:${OS_VERSION}

FROM ollama/ollama:latest AS ollama

FROM ${BASE_IMAGE}

# Copy ollama binary and libs
COPY --from=ollama /bin/ollama /usr/bin/ollama
COPY --from=ollama /usr/lib/ollama /usr/lib/ollama

# Do everything else as the opencog user.
USER opencog
WORKDIR /home/opencog

COPY /scripts/ /home/opencog

# Adjust per your personal setup.
# View GPU ram with `nvidia-smi` or with `nvtop`
#
# Pull several models. Numbers in paren are the capability scores.
#
#   The `ornith-1.5:397b` (62.47) is a 242GB download, 256K ctxt
#   The `gemma4:31b` (54.39) is a 19GB download, 256K ctxt
#   The `bonsai-2:27b` (53.95) is a 8.3GB download, 256K ctxt
#   The `qwen3.6:27b` (48.13) is an 18GB download, 256K ctxt
#   The `granite4.2:8b` (36.78) is a 5.3GB download, 128K ctxt
#   The `gemma4:12b` (35.48) is a 7.7GB download, 256K ctxt
#   The `ornith-1.5:9b` (34.04) is a 6.6GB download, 256K ctxt
#   The `nomic-embed-text` is small (300 MB), good for embeddings.
#
RUN ollama serve & \
    PID=$! && \
    until ollama list > /dev/null 2>&1; do sleep 1; done && \
    ollama pull bonsai-2:27b ; \
    ollama pull nomic-embed-text ; \
    kill $PID 2>/dev/null; true

EXPOSE 11434
EXPOSE 18082
EXPOSE 18083

# Run tini as PID1 and bash as PID2. tini will manage signals, etc.
ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["/bin/bash"]

ONBUILD USER root
