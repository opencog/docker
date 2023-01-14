#! /bin/bash
#
# Stop any running containers previously started with 'run.sh'
#
TAINER=`docker ps |grep base-learn |cut -f1 -d" "`
if test x"$TAINER" != x; then
   echo -n 'Stopping leftover container... '
   docker stop -t 1 $TAINER
fi
TAINER=`docker ps -a |grep base-learn`
if test x"$TAINER" != x; then
   echo -n 'Removing... '
   docker rm base-learn
fi
