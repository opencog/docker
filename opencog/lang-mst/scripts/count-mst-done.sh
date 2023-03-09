#! /bin/bash
#
# count-mst-done.sh
#
# Wait until MST counting and marginals computation is done,
# and then stop the docker container.
#
# -----------

sleep 60

# Wait for evidence that marginals have been computed.
while [[ ! -f /tmp/mst-marginals-done ]] ; do
	sleep 60
done

# Shut down the tmux session.
tmux kill-session

# Stop the main entry point.  We would like to do `kill -9 1` but
# docker does not allow PID 1 to be killed. But PID 1 is just
# running /bin/sh bash so once bash exits, then so will PID 1.
pkill -9 bash
