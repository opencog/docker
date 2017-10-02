#!/bin/bash
#
# This does what docker-compose does until docker-compose support for windows
# imporves. Ofcourse it can be used instead of docker-compose on unix systems

# TODO: move all this to octool

# Environment variable used by the commands edit them to suite your
export OPENCOG_SOURCE_DIR=$HOME/path/to/opencog
export RELEX_SOURCE_DIR=$HOME/path/to/relex
export ATOMSPACE_SOURCE_DIR=$HOME/path/to/atomspace
export COGUTIL_SOURCE_DIR=$HOME/path/to/cogutil
export MOSES_SOURCE_DIR=$HOME/path/to/moses

# NOTE: Error outputs during creation are silenced b/c this script is a
# temporary hack :-D
docker create --name postgres opencog/postgres &> /dev/null

docker create  \
    --name relex \
    -v "$RELEX_SOURCE_DIR:/relex" \
    -p "4444:4444" \
    -p "9000:9000" \
    -p "3333:3333" \
    -w /relex \
    -it opencog/relex /bin/sh -c "ant clean && ant  && ./opencog-server.sh" &> /dev/null

docker create \
    --name opencog_dev \
    -v "$OPENCOG_SOURCE_DIR:/opencog" \
    -v "$RELEX_SOURCE_DIR:/relex" \
    -v "$ATOMSPACE_SOURCE_DIR:/atomspace" \
    -v "$COGUTIL_SOURCE_DIR:/cogutil" \
    -v "$MOSES_SOURCE_DIR:/moses" \
    -e PYTHONPATH=/usr/local/share/opencog/python:/opencog/opencog/python/:/opencog/build/opencog/cython:/opencog/opencog/nlp/anaphora \
    -e OPENCOG_SOURCE_DIR=/opencog \
    -e PGHOST=db \
    -e PGUSER=opencog_user \
    -p "5000:5000" \
    -p "17001:17001" \
    -p "18001:18001" \
    -p "8080:8080" \
    -w /opencog \
    --link postgres:db \
    --link relex:relex \
    -it opencog/opencog-dev:cli &> /dev/null

# Keep the order of the start
echo "Starting postgres and relex"
docker start relex postgres
echo "Starting opencog_dev in 10 seconds"
sleep 10
docker start -i opencog_dev

# Cleanup on exit from opencog_dev
echo "Stopping postgres and relex"
docker stop relex postgres
echo "If you want to remove the containers run 'docker rm postgres relex'"
