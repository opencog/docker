
Dmitry head
===========

Basic dmitry head demo.

Run this by saying:
```
sudo docker run --rm --privileged -i \
   -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 \
   -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm \
   -e DISPLAY=:0.0 -t hansonrobotics/dmitry
```

