#! /bin/bash
#
# Run the docker container. Stop any previously running copies.
#
# ./stop.sh

MST_CONTAINER=mst-counter

TAINER=`docker ps |grep $MST_CONTAINER |cut -f1 -d" "`
if test x"$TAINER" != x; then
   echo "Container $MST_CONTAINER already running! Attaching to it..."
   # docker stop -t 1 $TAINER
	docker exec -it $MST_CONTAINER /bin/bash
else
	STAINER=`docker ps -a |grep $MST_CONTAINER |cut -f1 -d" "`
	if test x"$STAINER" != x; then
		echo "Container $MST_CONTAINER already exists! Starting it..."
	else
		docker create --name $MST_CONTAINER -p 17003:17003 -it opencog/lang-mst
	fi
	docker start -i $MST_CONTAINER
fi
