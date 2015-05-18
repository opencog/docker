# Continuese Integration

These directory carries ockerfiles orchasteration
The directories slave/precise and slave/utopic are kept for historic reasons.
The buildbot @ http://buildbot.opencog.org:8010/ only uses slave/latest

### Steps
1. ./set-workspace.sh   # If this is your first time then run this script. It
   configures the space for the buildbot to work in, within this directory. These directories are shared with the running containers.
2. docker-compose up -d   # This runs the containers in detached mode
3. docker-compose stop   # To stop all the containers

### TODO:
1. Add relex unit testing
2. Add integration testing between opencog and relex
3. Add integration testing between opencog and unity3d-opencog-game
