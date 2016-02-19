eva-owyl
========

Eva is an animated female head, capable of seeing you via a webcam,
and engaging you in social interaction, making eye contact, and
expressing her pleasure with smiles and looks of surprise and happiness.

There are several Eva demos.  This is a stand-alone demo of the basic
Hanson Robotics Eva blender rig, showing the full range of emotional
facial expressions, ranging from happiness to frustration, excitement
to boredom, as well as gestures such as shakes, nods, blinks and a
keep-alive breathing cycle.

This docker image includes vision processing and basic human-face
awareness behavior, but without a chatbot or any OpenCog processing
software.  The behaviors are controlled by a simple behavior-tree
script implemented using Owyl.  When the docker image is started, the
demo starts and runs automatically; she should be able to see you
(and your guests) through your webcam, and interact with all of you
in a most charming way.  This demo is roughly representative of the
state of the Hanson Robotics Eva development, as of April 2015.

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
