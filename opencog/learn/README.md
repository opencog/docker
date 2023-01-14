opencog/learn
-------------
This docker image contains all of the tools needed for the "learn"
subystem, including the nlp subsystem.

## Building

There are several ways to build:
* Run `../docker-build.sh -a` and then `../docker-build.sh -l`
  where `docker-build.sh` is the shell script in the parent directory.

* Run the `./build.sh` file in this directory.  You need to have
  built the base AtomSpace image (in the `../atomspace` directory) first.

To get fresh containers after github sources have changed, you will
need to add the `--no-cache` flag to the `docker build` command.
