#! /bin/bash
#
# tmux.sh
#
# Run some tmux (byobu) terminals.
#
# Use byobu so that the scroll bars actually work
byobu new-session -d -n 'bash' '$SHELL'

tmux new-window -n 'b2' '$SHELL'
tmux new-window -n 'b3' '$SHELL'
tmux new-window -n 'b4' '$SHELL'

# Spare-usage shell
tmux new-window -n 'bash' '$SHELL'

# Fix the annoying byobu display
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach
