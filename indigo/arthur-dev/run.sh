#! /bin/bash
#
echo -n 'Stopping leftover container.. '
docker stop -t 1 hansonrobotics-arthur-dev
echo -n 'Removing.. '
docker rm hansonrobotics-arthur-dev
xhost +
docker run --name="hansonrobotics-arthur-dev" --privileged  \
   -e DISPLAY=:0.0 \
   -p 33433:33433 -p 80:80 -p 9090:9090 \
   -v /dev/video0:/dev/video0 -v /dev/snd:/dev/snd  \
   -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri \
   -v /dev/shm:/dev/shm -it hansonrobotics/arthur-dev
xhost -
