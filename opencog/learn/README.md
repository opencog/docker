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

## Running

You can start the bare container by saying `./run.sh` in this directory.
However, it might be more interesting to try the word-pair-counting
experiment in the `../lang-pairs` directory.

If you do run this container, then the first thing you'll want to do
after starting it is to run the `setup-for-experiments.sh` shell script.
This will initialize a reasonable working environment for doing learning
experiments.
