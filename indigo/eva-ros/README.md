eva-ros
=======

An image containing the Hanson Robotics Eva blender rig, and the ROS
vision subsystem.  It does not contain any parts of OpenCog, or a
chatbot, nor any behavior controls; its just blender+vision.

The data processing pipeline consists of:
* A ROS node that takes video input from a UVC-compatible USB webcam
  (most typical desktop/laptop cameras are compatible).
* A ROS node that does face detection: it tries to find human faces in
  the video feed.
* A ROS node that maps the face postions to a 3D coordinate system.
* A ROS node that runs the Eva blender rig itself (i.e. uses blender
  to draw/animate Eva).
* A ROS node that converts PAU (Physiological Action Units) into motor
  controls, needed for animating physical robot models.

## Building

Run the `build.sh` file in this directory.  You need to have built
the ros-indigo-blender image (in the `../blender` directory) first.

## Testing
To verify that blender works, run the `run.sh` shell script.
This will start blender and the half-dozen ROS nodes that control it.
You should see a living, breating Eva; however, she is lobotimized:
there's no brain, she cannot actually see you, and will not react.
