#! /bin/bash
#
echo -n 'Stopping leftover container.. '
docker stop -t 1 hansonrobotics-eva
echo -n 'Removing.. '
docker rm hansonrobotics-eva
