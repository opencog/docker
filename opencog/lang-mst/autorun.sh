#! /bin/bash
#
# autorun.sh
#
# Fully-automated MST/MPG parsing. Assumes pair counting has been
# done previously, and that both text and data files are in the
# default locations.
#
# Creates a new container, if none exists, otherwise reuses an existing
# container. Copies text data into it, performs MST counting, computes
# marginals, and copies out the resulting dataset.
#
# Expects to find a text corpus in the `input-pages/pair-counted` directory.
# Expects a databse with word-pair marginals in `data/mst-parse.rdb`
#
# If used with the `-u` flag, then an existing container will be updated
# with additional counting. Useful for splitting up large counting runs
# into several stages.
#
# -----------------

MST_CONTAINER=mst-counter-auto
TEXT_DIR=text
DATA_DIR=data

UPDATE=$1

if [[ -z "$(ls $TEXT_DIR)" ]]; then
	echo "Error: The $TEXT_DIR directory appears to be empty!"
	exit 1
fi

if [[ -z "$(ls $DATA_DIR)" ]]; then
	echo "Error: The $DATA_DIR directory appears to be empty!"
	exit 1
fi

# Avoid trashing current work.
TAINER=`docker ps |grep $MST_CONTAINER |cut -f1 -d" "`
if test x"$TAINER" != x; then
   echo "Container $MST_CONTAINER is already running!"
	echo "Running containers cannot be updated."
	echo "To stop the existing container, do this:"
   echo "    docker stop -t 1 $MST_CONTAINER"
	echo "If you want a fresh container, then you must also do this:"
   echo "    docker rm $MST_CONTAINER"
	exit 1
fi

TAINER=`docker ps -a |grep $MST_CONTAINER |cut -f1 -d" "`
if test x"$TAINER" == x; then
	# Start Fresh
	echo "Creating container $MST_CONTAINER"
	docker create --name $MST_CONTAINER -it opencog/lang-mst
	docker container cp $DATA_DIR/. $MST_CONTAINER:/home/opencog/data

elif test x"$UPDATE" == x-u; then
	echo "Re-using existing container $MST_CONTAINER"
else
   echo "The container $MST_CONTAINER already exists!"
	echo "If you want to update it with additional data,"
	echo "then run this script with the -u flag."
	echo "If you want to start fresh, then remove it, like so:"
   echo "    docker rm $MST_CONTAINER"
	exit 1
fi

date
docker container cp $TEXT_DIR/. $MST_CONTAINER:/home/opencog/text

echo "Starting container $MST_CONTAINER"
docker start $MST_CONTAINER
docker exec -u 0 $MST_CONTAINER chown -R opencog:opencog /home/opencog/text
docker exec -u 0 $MST_CONTAINER chown -R opencog:opencog /home/opencog/data
docker exec -d $MST_CONTAINER /home/opencog/count-mst.sh
sleep 10

echo "Waiting for MST counting to finish in container $MST_CONTAINER"
echo "Caution: this may take hours, days or weeks!"
echo "If you're nervous and can't stand the wait, try this:"
echo "   docker exec -it $MST_CONTAINER /bin/bash"
echo "   tmux attach"
echo "and then prowl around in tmux with F3 and F4"
echo "This script will not exit until all processing is done."

docker exec $MST_CONTAINER /home/opencog/count-mst-done.sh

echo "Done MST counting in container $MST_CONTAINER"

echo "Copying MST dataset to $DATA_DIR"
docker container cp $MST_CONTAINER:/home/opencog/data/. $DATA_DIR
docker container cp $MST_CONTAINER:/home/opencog/text/. $TEXT_DIR
echo "Done!"
date
