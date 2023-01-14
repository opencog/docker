#! /bin/bash
#
# Run the docker container. Stop any previously running copies.
#
./stop.sh

docker run --name="base-learn" -it opencog/learn

clear
