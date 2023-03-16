#! /bin/bash
#
# autorun.sh
#
# Fully-automated word-pair counting.  Creates a new container, if none
# exists. Copies text data into it, performs pair-counting, computes
# marginals, and copies out the resulting dataset.
#
# Expects to find a text corpus in the `input-pages` directory.
#
# If used with the `-u` flag, then an existing container will be updated
# with additional counting. Useful for splitting up large word-pair
# counting runs into several stages.
#
# -----------------

# INPUT_DIR must match `run-config/2-pair-conf.sh`
INPUT_DIR=input-pages
TEXT_DIR=text
DATA_DIR=data

PAIR_CONTAINER=pair-counter-auto
UPDATE=$1

if [[ -z "$(ls $INPUT_DIR)" ]]; then
	echo "Error: You forgot to put an input corpus into the $INPUT_DIR directory!"
	exit 1
fi

# Avoid trashing current work.
TAINER=`docker ps |grep $PAIR_CONTAINER |cut -f1 -d" "`
if test x"$TAINER" != x; then
	echo "Container $PAIR_CONTAINER is already running!"
	echo "Running containers cannot be updated."
	echo "To stop the existing container, do this:"
	echo "    docker stop -t 1 $PAIR_CONTAINER"
	echo "If you want a fresh container, then you must also do this:"
	echo "    docker rm $PAIR_CONTAINER"
	exit 1
fi

TAINER=`docker ps -a |grep $PAIR_CONTAINER |cut -f1 -d" "`
if test x"$TAINER" == x; then
	# Start Fresh
	echo "Creating container $PAIR_CONTAINER"
	docker create --name $PAIR_CONTAINER -it opencog/lang-pairs
elif test x"$UPDATE" == x-u; then
	echo "Re-using existing container $PAIR_CONTAINER"
else
	echo "The container $PAIR_CONTAINER already exists!"
	echo "If you want to update it with additional pair data,"
	echo "then run this script with the -u flag."
	echo "If you want to start fresh, then remove it, like so:"
	echo "    docker rm $PAIR_CONTAINER"
	exit 1
fi

date

# Copy $INPUT_DIR as a whole. Note that `run-config/2-pair-conf.sh`
# expects input located at `input-pages` and `run-config/0-pipeline.sh`
# sets the basedir to `/home/opencog/text/`
docker container cp $INPUT_DIR $PAIR_CONTAINER:/home/opencog/text/

echo "Starting container $PAIR_CONTAINER"
docker start $PAIR_CONTAINER
docker exec -u 0 $PAIR_CONTAINER chown -R opencog:opencog /home/opencog/text
docker exec -d $PAIR_CONTAINER /home/opencog/count-pairs.sh
sleep 10

echo "Waiting for pair counting to finish in container $PAIR_CONTAINER"
echo "Caution: this may take hours, days or weeks!"
echo "If you're nervous and can't stand the wait, try this:"
echo "   docker exec -it $PAIR_CONTAINER /bin/bash"
echo "   tmux attach"
echo "and then prowl around in tmux with F3 and F4"

docker exec $PAIR_CONTAINER /home/opencog/count-pairs-done.sh

echo "Done pair counting in container $PAIR_CONTAINER"

echo "Copying word-pairs dataset to $DATA_DIR"
docker container cp $PAIR_CONTAINER:/home/opencog/text/. $TEXT_DIR
docker container cp $PAIR_CONTAINER:/home/opencog/data/. $DATA_DIR
echo "Done!"
date
