#!/bin/bash

source /.bash_profile
echo "Starting... this will take 15-20 seconds..."
byobu new-session -d -n 'roscore' 'roscore; $SHELL'
# byobu new-window -n 'blender' 'cd /catkin_ws/src/robo_blender/src && blender robo.blend --enable-autoexec --python  startup.py; $SHELL'
sleep 2;
byobu new-window -n 'blender' 'cd /catkin_ws/src/robo_blender/src && blender dmitry-mesh.anim_test.blend --enable-autoexec --python startup.py; $SHELL'
sleep 6;
byobu new-window -n 'cmdline' 'echo \"going into animation mode ...\"; rostopic pub --once /cmd_blendermode std_msgs/String Animations; clear; cat /catkin_ws/demo-hints.txt; $SHELL'

# Fix the annoying byobu display
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach

echo "Started"
$SHELL
