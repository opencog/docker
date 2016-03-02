#! /bin/bash
#
# eva.sh
#
# ROS + blender launch script for the Hanson Robotics Eva blender head.
# This shell script is automatically started within the docker container.
# It needs to run in the catkin_ws directory where the various ROS nodes
# and blender models were installed. It assumes that catkin_make was
# already run.

source devel/setup.sh
echo "Starting... this will take 15-20 seconds..."

# Use byobu so that the scroll bars actually work.
byobu new-session -d -n 'ros' 'roscore; $SHELL'
sleep 4;

# Run the relex parse server.
tmux new-window -n 'rlx' 'cd /opencog/relex && ./opencog-server.sh ; $SHELL'

# Single Video (body) camera and face tracker.
tmux new-window -n 'trk' 'roslaunch robots_config tracker-single-cam.launch; $SHELL'

# Publish the geometry messages. This includes tf2 which holds
# the face locations.
tmux new-window -n 'geo' 'roslaunch robots_config geometry.launch gui:=false; $SHELL'

### Start the blender GUI.
tmux new-window -n 'eva' 'cd /catkin_ws/src/blender_api && blender -y Eva.blend -P autostart.py; $SHELL'

# Start the IRC chatbot bridge.
tmux new-window -n 'irc' 'cd /opencog/opencog/build/opencog/nlp/irc && ./cogita -n ieva -f "Robot Eva" -t 17020; ; $SHELL'

# Start the cogserver.
# It seems to take more than 5 seconds to load all scripts!?
tmux new-window -n 'cog' 'cd /opencog/ros-behavior-scripting/src && guile -l btree-eva.scm ; $SHELL'
sleep 5

# Run the new face tracker.
tmux new-window -n 'fac' 'cd /opencog/ros-behavior-scripting/face_track && ./main.py ; $SHELL'

# Spare-usage shell
tmux new-window -n 'bash' '$SHELL'

# Fix the annoying byobu display.
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach

echo "Started"
