# Step to take
# 1. docker build --no-cache -t opencog/cogserver .
# 2. docker run -p 17001:17001 -it opencog/cogserver
# 3. rlwrap telnet localhost 17001

# Starting with the container with everything in it...
FROM opencog/opencog-dev:cli

# Docker defaults
WORKDIR /home/opencog/
USER opencog

## Start cogserver when container runs
CMD /usr/local/bin/cogserver
