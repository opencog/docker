#! /bin/bash
#
# Stop any running containers previously started with 'run.sh'
#
TAINER=`docker ps |grep opencog-indigo-base |cut -f1 -d" "`
if test x"$TAINER" != x; then
	echo -n 'Stopping leftover container.. '
	docker stop -t 1 $TAINER
fi
TAINER=`docker ps -a |grep opencog-indigo-base`
if test x"$TAINER" != x; then
	echo -n 'Removing.. '
	docker rm opencog-indigo-base
fi
