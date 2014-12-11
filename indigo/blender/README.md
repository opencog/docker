ros-indigo-blender
==================

An integrated demo container for using ROS to control blender animations.
The container holds only the base packages needed for running blender
and ROS together; no actual ROS nodes are started, no blender animations
are run.

## Building

The build the docker container, say this:
`docker build -t opencog/ros-indigo-blender .`

## Testing
To verify that blender works, try this:
```
# Allow the docker container to access the local display
xhost +

# Run the docker containaer
docker run --rm --privileged -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 \
  -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -e DISPLAY=:0.0 \
  -t opencog/ros-indigo-blender 

# Within the docker container, run blender
blender

# Exit the docker container
exit

# Disable excessively permissive X11 connection permissions
xhost -
```

