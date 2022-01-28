ros-opencog
============

A docker image that installs both ROS and OpenCog.  The image holds
only the base packages needed for running OpenCog and ROS together;
The CogServer is not started, no ROS nodes are started.

## Building

Run the `build.sh` file in this directory.  You need to have built
the base ROS image (in the `../ros-base` directory) first.

## Testing
To verify that OpenCog works, run the `run.sh` shell script.
This will create a tmux (byobu) session inside the docker container.
There are several things you can try:

* Navigate to the build directories in `/opencog` and run the unit
  tests there.

* Start `guile` at the prompt, and then try `(use-modules (opencog))`

* Try starting the cogserver! At the guile prompt:
```
(use-modules (opencog) (opencog cogserver))
(start-cogserver)
(Concept "foobar" (stv 0.5 0.8))
```
Then from another tmux prompt:
```
telnet localhost 17001
(use-modules (opencog))
(cog-node 'ConceptNode "foobar")
```
The above should display `(ConceptNode "foobar" (stv 0.5 0.80000001))`.
of course!
