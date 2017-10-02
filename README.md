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
and bodies, mostly those from Hanson Robotics. Several of the heads are
modeled with blender, and so can be usefully worked with and controlled
even without a physical robot.

These files are contained in the `hydro` and [indigo](indigo)
directories. Eva comes in two forms: a basic animation and social
interaction demo, in the [indigo/eva-owyl](indigo/eva-owyl) folder,
and a full-featured system (under heavy development, and probably broken)
in the [indigo/eva-opencog](indigo/eva-opencog) folder.  See the
README's in those directories for more info.

## Dockerfiles for OpenCog
Opencog system dockerfiles can be found in the opencog and buildbot
directories.  See [opencog's README.MD](opencog/README.md)
and [buildbot's README.md](buildbot/README.md).

The Dockerfiles in the directories `opencog/tools/distcc`,
`opencog/embodiment` and `opencog/cogserver` are not detailed because
they are not in active use.

### Docker image structure:

    ├─opencog/opencog-deps:utopic
    ├─opencog/opencog-deps:latest
      ├─buildbot_* (Where * = atomspace, cogutil, opencog, moses)
      ├─opencog/cogutil:latest
        ├─opencog/opencog-dev:cli (for a dev environment)
        ├─opencog/opencog-dev:ide
        ├─opencog/moses

    ├─opencog/relex

### Organizational Notes:

* `opencog/opencog-deps:utopic`: ubuntu 14.10 based image with all OpenCog's
   dependencies installed.

* `opencog/opencog-deps:latest`: ubuntu 14.04 based image with all OpenCog's
   dependencies installed. This forms the base of opencog/cogutil. It
   likely will be updated to the latest LTS as it is released. Has some command
   line tools for use by developers.

* `buildbot_*`: Is used for buildbot found [here](buildbot.opencog.org:8010)

* `opencog/cogutil`: It is the base image for `opencog/opencog-dev:cli`,
   `opencog/opencog-dev:ide` and `opencog/moses` images. It installs cogutil
   over `opencog/opencog-deps` image. The main reason for having this is to
   speed up rebuilds as one doesn't ave to rebuild the `opencog-deps` image,
   unless there are dependency changes, and rebuilding this image will suffice
   for updating the dependent images.

* `opencog/opencog-dev:cli`: Mainly for running/developing through a shared
   filesystem between host and container.

* `opencog/opencog-dev:ide`: To be used for developing using ides. QtCreator
   is installed.

* `opencog/moses`: It has moses and R installed. R is installed for those
   who want to use the R binding for moses. The binding is not yet
   included but can be found [here](https://github.com/mjsduncan/Rmoses).

* `opencog/relex`: It is a self-contianed image for running relex and
   link-grammar servers.

## Usage
* To run the demos and other containers, docker must be installed.
  Instructions can be found [here](https://docs.docker.com/installation/).
  The [Giving non-root access](https://docs.docker.com/installation/ubuntulinux/#giving-non-root-access)
  section on the page explains how to avoid having to use `sudo` all the time.

* The docker-build.sh file in opencog directory is used for building
  some of the images. Run `./docker-build.sh -h` for viewing available
  options.

* To use docker-compose follow the instruction in the README file in the
  opencog directory.
