#! /bin/bash
#
# stop.sh
#
# Usage: ./stop.sh
#
# This will stop and remove a running docker container started with run.sh
#
echo -n 'Stopping leftover container.. '
docker stop -t 1 hr-arthur-devel
echo -n 'Removing.. '
docker rm hr-arthur-devel
