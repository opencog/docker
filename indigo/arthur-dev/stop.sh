#! /bin/bash
#
echo -n 'Stopping leftover container.. '
docker stop -t 1 hansonrobotics-arthur-dev
echo -n 'Removing.. '
docker rm hansonrobotics-arthur-dev
