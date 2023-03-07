#! /bin/bash
#
# Fully-automated word-pair counting. Assumes the text corpus is located
# in the text directory.
#
# -----------------

PAIR_CONTAINER=pair-counter-auto
INPUT_DIR=input-pages
TEXT_DIR=text
DATA_DIR=data

if [[ -z "$(ls $TEXT_DIR)" ]]; then
	echo "Error: You forgot to put an input corpus into the $TEXT_DIR directory!"
	exit 1
fi

# Get rid of earlier instances. Hope they didn't have much in them!
TAINER=`docker ps |grep $PAIR_CONTAINER |cut -f1 -d" "`
if test x"$TAINER" != x; then
   echo "Stopping leftover container $PAIR_CONTAINER"
   docker stop -t 1 $TAINER
fi

TAINER=`docker ps -a |grep $PAIR_CONTAINER |cut -f1 -d" "`
if test x"$TAINER" != x; then
   echo "Removing old container $PAIR_CONTAINER"
   docker rm $TAINER
fi

# Start fresh
echo "Creating container $PAIR_CONTAINER"
docker create --name $PAIR_CONTAINER -it opencog/lang-pairs
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
docker container cp $PAIR_CONTAINER:/home/opencog/text $TEXT_DIR
docker container cp $PAIR_CONTAINER:/home/opencog/data $DATA_DIR
echo "Done!"
