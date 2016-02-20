#! /bin/bash
#
# Stop any running containers previously started with 'run.sh'
#
TAINER=`docker ps |grep opencog-incog-blender |cut -f1 -d" "`
if test x"$TAINER" != x; then
	echo -n 'Stopping leftover container.. '
	docker stop -t 1 $TAINER
fi
TAINER=`docker ps -a |grep opencog-incog-blender`
if test x"$TAINER" != x; then
	echo -n 'Removing.. '
	docker rm opencog-incog-blender
fi

# Disable local X11 connections, just in case.
xhost -local:root
