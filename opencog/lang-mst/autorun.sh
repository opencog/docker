#! /bin/bash
#
# Fully-automated MST/MPG parsing. Assumes pair counting has been
# done previously, using the default docker container for pair counting.
#
# -----------------
#
mkdir tmp
cd tmp
docker container cp pair-counter:/home/opencog/text .
docker container cp pair-counter:/home/opencog/data .

# This will fail ugly, if the container already exists.
docker create --name mst-counter -it opencog/lang-mst
docker container cp text mst-counter:/home/opencog/
docker container cp data mst-counter:/home/opencog/

