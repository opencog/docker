#! /bin/bash
#
# Stop any running containers previously started with 'run.sh'
#
TAINER=`docker ps |grep eva-ros |cut -f1 -d" "`
if test x"$TAINER" != x; then
	echo -n 'Stopping leftover container... '
	docker stop -t 1 $TAINER
fi
TAINER=`docker ps -a |grep eva-ros`
if test x"$TAINER" != x; then
	echo -n 'Removing... '
	docker rm eva-ros
fi

# Disable local X11 connections, just in case.
xhost -local:root
