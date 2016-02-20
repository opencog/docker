ros-indigo-base
===============
Docker files for Hanson Robotics robots and heads.  The most
sophisticated of the bunch is Eva, an animated female head, capable
of seeing you (via a webcam), talking to you (via a chatbot), and
engaging you (and your guests) in social interaction.  She will
make eye contact, and express her pleasure with smiles and looks
of surprise and happiness.

![Eva Splash 1](Eva-1-small.png) ![Eva Splash 2](Eva-2-small.png) ![Eva Splash 3](Eva-3-small.png)

## Hierarchy and dependents

The docker image hierarchy is:

    ├─ros-indigo-base
      ├─ros-indigo-blender
        ├─eva-owyl
        ├─eva-ros
        ├─ros-arthur-animation
        ├─ros-arthur-dev
        ├─ros-indigo-opencog
      ├─ros-indigo-dev
        ├─ros-indigo-einstein
        ├─ros-indigo-zenorsm


* `base` contains a docker image defining only the basic ROS indigo
   nodes shared by all robots, and nothing more.

* `blender` contains a docker image for ROS and blender. Depends on
   base, above.  it does not provide anything beyond a configured
   ROS+blender environment.

* `eva-ros` contains the basic Hanson Robotics Eva blender rig,
   together with the ROS nodes needed for vision processing.
   Upon startup, it will automatically run the blender rig and the
   vision subsystem.  However, it is missing the chatbot and the
   behavior subsystem; Eva will stare blankly into space and breath,
   but do nothing more.

* `eva-owyl` is a a stand-alone demo of the basic Hanson Robotics
   Eva blender rig, showing the full range of emotional facial
   expressions, ranging from happiness to frustration, excitement
   to boredom, as well as gestures such as shakes, nods, blinks and
   a keep-alive breathing cycle. It includes basic vision processing
   and basic human-face awareness behavior, but without a chatbot or
   any OpenCog processing software.  This is a stand-alone demo,
   without any other dependencies. It is representative of the state
   of development of the Hanson Robotics Eva blender rig, as of
   May 2015.


## Miscellaneous packages

* `dev`, derived from `base`, contains a docker image for a ROS indigo
   development environment.

* `einstein` container for the Hanson Robotics' small Einstein head.
  This container was demoed at ROS Kong 2014 by David Hanson.

* `zenorsm` ... Zeno ?? with Einstein head ??

## Unmaintained images
The arthur-animation and arthur-dev packages are deprecated. They
implement an older animation system that was difficult to work with.
Most importantly, eye-tracking and emotion gestures were not blended
and integrated in an easy-to-use fashion.

* `arthur-animation` contains a demo for the Hanson Robotics Arthur head,
   showing how ROS messages can be used to control facial animations.
   That is, the blender rig is encapsulated in a ROS node; the rig
   reacts to ROS messages.

* `arthur-dev` contains the full end-to-end development environment
   for the Hanson Robotics Arthur head.  This includes a half-dozen
   ROS nodes for camera and vision processing, scripted behavior trees,
   motor controllers, and a web-based user interface.

## Building
Most subdirectories contain a `build.sh` script for building the
particular docker image.

Use `build-all.sh` to build the Eva docker image and it's dependents.
This may take an hour or more.

## Running
Most subdirectories contain a pair of scripts: `run.sh` and `stop.sh`.
These will run and stop the containers defined in that directory.

# Design
Building a well-designed system using docker and ROS is impossible at
this point in time (as of spring 2016). This is because the way that
Docker does networking is in direct conflict with how ROS does
networking.  Worse, the way that Docker does networking is changing
(for Swarm), while ROS will also change, with a completely different
messaing sysem for ROS2.  It will be a while before the dust settles
and there's a coherent networking policy.

There are three reasonable design choices:

* Put OpenCog and ROS in the same Docker container.   This is ugly
  and unpleasant, and not a reasonable design choice at all, but it's
  the one we take, because nothing else works.

* Put Eva blender and vision in one Docker container, (or maybe two
  containers, one for blender, and one for vision) and OpenCog in a
  third container. This is an ideal design, but cannot be made to work
  due to conflicting networking models.

* Put every ROS node into its own container. This would result in a
  half-dozen to a dozen or more containers, and becomes a manageability
  issue: too many containers to correctly build, set up, monitor, and
  run.  Might work for some super-admin with superman-cloud-fu, but is
  way too complicated for us AI scientists.  Also, at this time, it also
  won't work, due to design limiations/flaws with Docker+ROS networking.

Below is a discussion and notes about the issues that block the second
and third design points.

Main issue: a ROS publisher inside a Docker container cannot be
subscribed to outside of a Docker container.  There are several ways
to set this up and try to hack around it.

## Version A: Default Docker networking.
Start the ROS container in directory `base` using the `./run.sh` command.
Make yourself a tmux session as follows (this is for convenience only):
```
byobu new-session -d -n 'aa' '$SHELL'
tmux new-window -n 'bb' '$SHELL'
tmux new-window -n 'cc' '$SHELL'
tmux new-window -n 'dd' '$SHELL'
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach
```
Start `roscore` by hand in one terminal.  In another terminal, publish
something:
```
rostopic pub -r 5 /foo std_msgs/String '{data: hallooooo}'
```
In a third terminal, subscribe:
```
rostopic echo /foo
```
Yayy! This works!  Can we subscribe to this topic outside of this
container? No, we cannot. Here's how to check.  Inside a terminal, say
`ifconfig` and get the IP address of the container. Its will probably be
`172.17.0.2`.  Alternately, you can get this number from the outside,
with
```
docker inspect opencog-eva-ros | grep IPAdress
```
We can now attempt to contact ROS from the outside world. First, set
```
export ROS_MASTER_URI=http://172.17.0.2:11311
```
Now, ROS will *almost* work. But not quite.  This works:
```
rostopic list
rostopic info /foo
```
This fails (it hangs):
```
rostopic echo /foo
```
The reason that this is broken is because ROS messages themselves contain
tcpip port numbers, and Docker uses a form of port proxying and
firewalling (iptables) that remaps ports. Thus, port numbers inside the
container do not match the port number contained in the message, when
that message is coming from outside the container.  This mis-match is at
the root of the hang.

Curiously, the reversed process works: if you publish on the outside,
then you can subscribe on the inside, just fine.

This issue prevents the natural separation between OpenCog and Eva-ROS.
The Eva-ROS container is publishing `/camera/face_locations` and OpenCog
subscribes to that to find out about the location of human faces visible
to Eva.  Since the messages never arrive, Eva is blind, and sees
nothing.

## Version B: New Docker networking.
Starting with Docker version 1.9 (the current version in Ubuntu 14.04
trusty LTS), docker introduces a networking overlay.  This seems like it
is almost enough to get ROS working in Docker, and there are even some
'famous' highly visible tutorials for this:

Its not
