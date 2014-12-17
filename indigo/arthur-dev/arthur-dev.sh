#!/bin/bash
# 
# arthur-dev.sh
#
# ROS + blender launch script for the Hanson Robotics animation demo
# This shell script is automatically started within the docker container.
# It needs to run in the catkin_ws directory where the various ROS nodes
# and blender models were installed. It assumes that catkin_make was
# already run.

source devel/setup.sh
echo "Starting... this will take 15-20 seconds..."

# Use byobu so that the scroll bars actually work
byobu new-session -d -n 'roscore' 'roscore; $SHELL'
sleep 2;
tmux new-window -n 'arthur' 'roslaunch robots_config arthur-dev.launch; $SHELL'
sleep 6;

# Start web server for providing a nice UI for the animations.
tmux new-window -n 'rosbridge' 'roslaunch rosbridge_server rosbridge_websocket.launch'
tmux new-window -n 'webserver' 'cd /catkin_ws/src/ros_motors_webui && python -m SimpleHTTPServer 80; $SHELL'

# One blender session for object tracking, another for animations
tmux new-window -n 'blndr-trk' 'cd /catkin_ws/src/robo_blender/src && blender robo.blend --enable-autoexec --python startup.py; $SHELL'
tmux new-window -n 'blndr-anim' 'cd /catkin_ws/src/robo_blender/src && blender animate-test.blend --enable-autoexec --python startup.py; $SHELL'

# Video camera and face tracker
tmux new-window -n 'camera' 'roslaunch ros2opencv uvc_cam.launch'
tmux new-window -n 'face' 'roslaunch pi_face_tracker face_tracker_uvc_cam.launch'

# Spare-usage shell
tmux new-window -n 'bash' '$SHELL'

# Fix the annoying byobu display
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach

echo "Started"
