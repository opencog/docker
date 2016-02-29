eva-opencog
===========

An image containing the OpenCog + Hanson Robotics Eva robot.
This is the whole kit-n-kaboodle. In development. Probably broken.

The visual processing+animation pipeline consists of:
* A ROS node that takes video input from a UVC-compatible USB webcam
  (most typical desktop/laptop cameras are compatible).
* A ROS node that does face detection: it tries to find human faces in
  the video feed.
* A ROS node that maps the face postions to a 3D coordinate system.
* A ROS node that runs the Eva blender rig itself (i.e. uses blender
  to draw/animate Eva).

The speech processing pipleing consists of:
* The RelEx server parsing English.
* The OpenCog chatbot (currently not attached to any input source).

## Building

Run the `./build.sh` file in this directory.  You need to have built
the `ros-incog-blender` image (in the `../ros-incog-blender` directory)
first.

## Testing
To verify that blender works, run the `./run.sh` shell script.
This will start blender and the half-dozen ROS nodes that control it.
You should see a living, breating Eva.

You can chat to Eva by going to the guile prompt, and typing in
chat text like so:
```
(process-query 'luser "Look left!")
(process-query 'luser "Smile!")
(process-query 'luser "Look sad!")
(process-query 'luser "Eva, express boredom")
```

Turn on the non-verbal behaviors by saying:
```
(run)
```

If things don't work, open a github issue or complain on the mailing list!

## TODO
* Remove joint_state_publisher robot_state_publisher from the geometry
  startup -- we only need to track face locations via geometry, and not
  the rest of the stuff.
