#!/bin/bash
echo "Starting"
tmux new-session -n 'einstein' -d ' roslaunch robots_config einstein.launch; $SHELL'
sleep 5;
export ROS_NAMESPACE=/einstein
tmux new-window -n 'rosbridge' 'roslaunch rosbridge_server rosbridge_websocket.launch; $SHELL'
tmux new-window -n 'webserver' 'cd /catkin_ws/src/ros_motors_webui && python -m SimpleHTTPServer; $SHELL'
tmux attach;
echo "Started"

