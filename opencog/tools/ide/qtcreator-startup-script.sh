#! /bin/bash
#
# This script is the container entry script used to setup cli
# interface and trigger qtcreator.

tmux new-session -d -n "opencog-development";
echo evaluating startup script... ;\
tmux set-option -g set-remain-on-exit on ;\
tmux bind-key R respawn-window ;\
tmux split-window -d -v -p 25 ;\
tmux send -t 1 qtcreator
tmux send -t 1 Enter
tmux attach
