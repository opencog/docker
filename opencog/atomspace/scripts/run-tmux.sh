#! /bin/bash
#
# run-tmux.sh
#
# Run tmux with byobu to multiplex multiple terminals.
# This just opens a bunch of terminals, and nothing more.
# The terminal names are just a convention that seems handy
# for a generic workflow.
#
# Use F3 and F4 to switch between terminals. `man byobu` for more.
#
#------------------

# Work around an lxc-attach bug.
if [[ `tty` == "not a tty" ]]
then
	script -c $0 /dev/null
	exit 0
fi

# Use byobu so that the scroll bars actually work
byobu new-session -d -s 'claude' -n 'cntl' \
	'echo -e "\nControl shell; you might want to run top here.\n"; $SHELL'

# Code
tmux new-window -n 'code-a' 'echo -e "\nCoding shell.\n"; $SHELL'

# Code
tmux new-window -n 'code-b' 'echo -e "\nCoding shell.\n"; $SHELL'

# Server
tmux new-window -n 'srv' \
	'echo -e "\nCogServer shell\n"; $SHELL'

# Spare
tmux new-window -n 'spare' 'echo -e "\nSpare-use shell.\n"; $SHELL'

# Fix the annoying byobu display
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach
