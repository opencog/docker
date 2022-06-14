#
# How to run the cogserver:
#
# 1. Build the container:
#
#    docker build --no-cache -t opencog/cogserver .
#
# 2. Run the container, exposing port 17001 to the external world:
#
#    docker run -p 17001:17001 -it opencog/cogserver
#
# 3. Wait until this is printed: `Listening on port 17001`
#
# 4. Connect into the cogserver:
#
#    rlwrap telnet localhost 17001
#

# Start with the container that has everything in it...
FROM opencog/opencog-dev:cli

# Docker defaults
WORKDIR /home/opencog/
USER opencog

## Start cogserver when container runs
CMD /usr/local/bin/cogserver
