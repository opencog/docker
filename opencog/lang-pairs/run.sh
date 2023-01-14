#! /bin/bash
#
# Run the docker container. Stop any previously running copies.
#
# ./stop.sh

# The -p flag is external_port:internal_port
docker create --name pair-counter -p 17001:17001 -it opencog/lang-pairs
docker start -i pair-counter

clear
