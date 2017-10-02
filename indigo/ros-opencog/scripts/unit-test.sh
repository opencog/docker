#! /bin/bash
#
# unit-test.sh
#
# Run the OpenCog unit tests on container startup.
# Provide a spare teminal in the meanwhile.
#
echo "Will Run OpenCog unit tests..."

# Use byobu so that the scroll bars actually work
byobu new-session -d -n 'bash' '$SHELL'

tmux new-window -n 'utils' 'cd /opencog/cogutil/build && make test; $SHELL'
tmux new-window -n 'atoms' 'cd /opencog/atomspace/build && make test; $SHELL'
tmux new-window -n 'servr' 'cd /opencog/opencog/build && make test; $SHELL'

# Spare-usage shell
tmux new-window -n 'bash' '$SHELL'

# Fix the annoying byobu display
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach

echo "Started"
