#! /bin/bash
#
# Fully-automated word-pair counting. Assumes the text corpus is located
# in the text directory.
#
# TODO:
# * check if container exists already and/or is running already.
# -----------------

PAIR_CONTAINER=pair-counter-auto
TEXT_SOURCE=text

if [[ -z "$(ls $TEXT_SOURCE)" ]]; then
	echo "Error: You forgot to put an input corpus into the $TEXT_SOURCE directory!"
	exit 1
fi

#
# This will fail ugly, if the container already exists.
echo "Creating container $PAIR_CONTAINER"
docker create --name $PAIR_CONTAINER -it opencog/lang-pairs
docker container cp $TEXT_SOURCE $PAIR_CONTAINER:/home/opencog

echo "Starting container $PAIR_CONTAINER"
docker start $PAIR_CONTAINER
docker exec -d $PAIR_CONTAINER /home/opencog/count-pairs.sh
sleep 10

echo "Waiting for pair counting to finish in container $PAIR_CONTAINER"
docker exec $PAIR_CONTAINER /home/opencog/count-pairs-done.sh
