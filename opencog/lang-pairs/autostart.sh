#! /bin/bash
#
# autostart.sh
#
# Create and run a word-pair counting container. Fully automated; starts
# it, runs to completion, and then copies out the resulting database.
# There must be a text corpus located in the `text` directory.
#
# -----------------

PAIR_CONTAINER=pair-counter-auto
INPUT_DIR=input-pages
TEXT_DIR=text
DATA_DIR=data

if [[ -z "$(ls $INPUT_DIR)" ]]; then
	echo "Error: You forgot to put an input corpus into the $INPUT_DIR directory!"
	exit 1
fi

# Avoid trashing current work.
TAINER=`docker ps |grep $PAIR_CONTAINER |cut -f1 -d" "`
if test x"$TAINER" != x; then
   echo "Container $PAIR_CONTAINER is already running!"
	echo "If you want to start fresh, then do this:"
   echo "    docker stop -t 1 $TAINER"
   echo "    docker rm $TAINER"
	exit 1
fi

TAINER=`docker ps -a |grep $PAIR_CONTAINER |cut -f1 -d" "`
if test x"$TAINER" != x; then
   echo "The container $PAIR_CONTAINER already exists!"
	echo "If you want to start fresh, then remove it, like so:"
   echo "    docker rm $TAINER"
	exit 1
fi

# Start Fresh
date
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
date
