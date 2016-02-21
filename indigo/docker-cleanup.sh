#! /bin/bash
#
# General docker cleanup. Handy if you've been ahcking and need the
# accumulated juk to be blown away.  Use at your own risk.
#
# First, remove the non-running containers.
# docker rm $(docker ps -aq -f "status=exited")
TAINERS=`docker ps -aq -f "status=exited" -f "status=dead"`
if test x"$TAINERS" != x; then
	echo -n 'Removing exited containers...'
	docker rm $TAINERS
fi

# Next, remove failed images.
# docker rmi $(docker images -q -f "dangling=true")
MAGES=`docker images -q -f "dangling=true"`
if test x"$MAGES" != x; then
	echo -n 'Removing dangling images'
	docker rmi $MAGES
fi

# docker rmi $(docker images --filter dangling=true)
