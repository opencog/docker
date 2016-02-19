eva-owyl
========

An integrated demo image for demoing the Hanson Robotics Eva blender
rig.  The image contains not only the Eva blender rig, but also a
half-dozen ROS nodes that control that rig.  When the image is started,
the demo runs automatically.

This demo does NOT include any OpenCog infrastructure: no AtomSpace,
no chatbot, on OpenCog behaviors.  This captures the state of
development, as of June 2015.

The demo processing pipeline consists of:
* A ROS node that takes video input from a UVC-compatible USB webcam
  (most typical desktop/laptop cameras are compatible).
* A ROS node that does face detection: it tries to find human faces in
  the video feed.
* A ROS node that maps the face postions to a 3D coordinate system.
* A ROS node that implements social interactions: when a face is seen
  for the first time, Eva makes eye contact and smiles. When a face
  disappears, Eva becomes at first disappointed, perhaps angry, and
  eventually bored.  In the meantime, the behavior controller cycles
  through a variety of suitable emotional expressions.
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
You should see a living, breating Eva who smiles at you and tries to
make and maintain eye contact as you move around.
