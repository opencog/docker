#! /bin/bash
#
# Fully-automated MST/MPG parsing. Assumes pair counting has been
# done previously, and that both text and data files are in the
# default locations.
#
# -----------------

MST_CONTAINER=mst-counter-auto
TEXT_DIR=text
DATA_DIR=data

if [[ -z "$(ls $TEXT_DIR)" ]]; then
	echo "Error: The $TEXT_DIR directory appears to be empty!"
	exit 1
fi

if [[ -z "$(ls $DATA_DIR)" ]]; then
	echo "Error: The $DATA_DIR directory appears to be empty!"
	exit 1
fi

date

# Get rid of earlier instances. Hope they didn't have much in them!
TAINER=`docker ps |grep $MST_CONTAINER |cut -f1 -d" "`
if test x"$TAINER" != x; then
   echo "Stopping leftover container $MST_CONTAINER"
   docker stop -t 1 $TAINER
fi

TAINER=`docker ps -a |grep $MST_CONTAINER |cut -f1 -d" "`
if test x"$TAINER" != x; then
   echo "Removing old container $MST_CONTAINER"
   docker rm $TAINER
fi

# Start fresh
echo "Creating container $MST_CONTAINER"
docker create --name $MST_CONTAINER -it opencog/lang-mst
docker container cp $TEXT_DIR $MST_CONTAINER:/home/opencog/
docker container cp $DATA_DIR $MST_CONTAINER:/home/opencog/

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

docker exec $MST_CONTAINER /home/opencog/count-mst-done.sh

echo "Done MST counting in container $MST_CONTAINER"

echo "Copying MST dataset to $DATA_DIR"
docker container cp $MST_CONTAINER:/home/opencog/data $DATA_DIR
docker container cp $MST_CONTAINER:/home/opencog/text $TEXT_DIR
echo "Done!"
date
