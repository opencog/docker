#!/bin/bash
#
# arthur.sh
#
# ROS + blender launch script for the Hanson Robotics animation demo
# This shell script is automatically started within the docker container.
# It needs to run in the catkin_ws directory where the various ROS nodes
# and blender models were installed. It assumes that catkin_make was
# already run.

source devel/setup.sh
echo "Starting... this will take 15-20 seconds..."
byobu new-session -d -n 'roscore' 'roscore; $SHELL'
sleep 2;
byobu new-window -n 'blender' 'cd /catkin_ws/src/robo_blender/src && blender animate-test.blend --enable-autoexec --python startup.py; $SHELL'
sleep 6;
byobu new-window -n 'cmdline' 'echo \"going into animation mode ...\"; rostopic pub --once /cmd_blendermode std_msgs/String Animations; clear; cat /catkin_ws/demo-hints.txt; $SHELL -l' 

# Fix the annoying byobu display
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach

echo "Started"
$SHELL
