[![publish](https://github.com/opencog/docker/actions/workflows/publish-images.yml/badge.svg)](https://github.com/opencog/docker/actions)

# OpenCog Docker library
This repository contains various docker images for demoing and using
various parts of the OpenCog project. Using the docker images here is
the easiest way to get interesting parts of OpenCog running.

By using Docker, most of the difficulty of installing and configuring
the large variety of required packages and dependencies is eliminated.
The individual docker files specify exactly what is needed to run a
system or demo.  Using these is as simple as saying `./build.sh` and
then `./run.sh`.

The most sophisticated demo is Eva, an animated female head, capable
of seeing you (via a webcam), talking to you (via a chatbot), and
engaging you (and your guests) in social interaction.  She will
make eye contact, and express her pleasure with smiles and looks
of surprise and happiness.

![Eva Splash 1](indigo/Eva-1-small.png) ![Eva Splash 2](indigo/Eva-2-small.png) ![Eva Splash 3](indigo/Eva-3-small.png)

Another particularly interesting container is that for the OpenCog to
MineCraft bridge, which allows OpenCog to operate an avatar running
around in the MineCraft world.

## Dockerfiles for Robot Operating System (ROS)
Dockerfiles for demoing and working with various different robot heads
and bodies, mostly those from
[Hanson Robotics](https://www.hansonrobotics.com/). Several of the
heads are modeled with blender, and so can be usefully worked with
and controlled even without a physical robot.

Sadly, the Eva demo has bitrotted. There have been some valiant efforts
to bring Eva back to life, to be found in the [noetic](noetic) folder,
but they stand incomplete. The primary roadblock is that Eva requires
an older version of [blender](https://www.blender.org/), version 2.79,
which dates back to 2016. Perhaps a blender expert could port Eva to
a newer version.

The ROS dockerfiles are contained in the [hydro](hydro),
[indigo](indigo), [noetic](noetic) directories. Eva came in
two forms: a basic animation and social interaction demo, in the
[indigo/eva-owyl](indigo/eva-owyl) folder, and a full-featured
system in the [indigo/eva-opencig](indigo/eva-opencog) folder.
These no longer work, due to bitrot; the "good stuff", the last
attempt to get everything working, is in the [noetic](noetic) folder.

## Dockerfiles for OpenCog
Opencog system dockerfiles can be found in the [opencog](opencog)
directory. Up-to-date, current pre-built versions of these Docker
containers can be downloaded from dockerhub. Simply go to the
[opencog](opencog) directory, and run the `docker-build.sh` shell
script. `docker-build.sh -h` for help, and `docker-build.sh -a`
to download everything.

### Obsolete demos
If you have been sent here, and cannot find what you were told about,
look in the [archive](archive) directory. It contains copies of all of
the unsupported, abandond and obsolete projects that were once a part of
OpenCog.

### OpenCog demo
The [lang-pairs](opencog/lang-pairs) container hosts a stand-alone
word-pair visualization demo. If you have some files containing
text, ideally more than a few thousand words and less than a few
million, then this demo can process the files and extract the
word-pair mutual information. This can then be viewed by aiming
a web browser at the docker container. Just follow the directions
in the README.

### Docker image dependency structure:

    ├─opencog/opencog-deps
      ├─opencog/atomspace
        ├─opencog/learn
          ├─opencog/lang-pairs
          ├─opencog/lang-mst

### Organizational Notes:
Dockerhub's copies of opencog dockerfiles are here:
https://hub.docker.com/search?q=opencog

* `opencog/opencog-deps:latest`: Ubuntu 22.04 based image with all
   OpenCog's dependencies installed. This does not need to be rebuilt,
   except to pick up the latest version of the base OS and OS security
   patches. This is the base image for `opencog/atomspace`.

* `opencog/atomspace`: This depends on the `opencog/opencog-deps:latest`
  image. It provides the AtomSpace, RocksDB, the Cogserver, and the
  network CogStorageNode, allowing complex AtomSpace networks to
  be built.

* `opencog/learn`: This depends on the `opencog/atomspace:latest`
  image. It provides the basic development environment for the
  language learning project.

## Usage
* To run the demos and other containers, docker must be installed.
  Instructions can be found [here](https://docs.docker.com/installation/).
  The [Giving non-root access](https://docs.docker.com/installation/ubuntulinux/#giving-non-root-access)
  section on the page explains how to avoid having to use `sudo` all the time.

* The `docker-build.sh` file in [opencog](opencog) directory can be used
  to build the containers mentioned above.  Run `./docker-build.sh -h`
  for usage instructions.
