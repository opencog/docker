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

## Dockerfiles for OpenCog
Opencog system dockerfiles can be found in the [opencog](opencog)
directory. Up-to-date, current pre-built versions of these Docker
containers can be downloaded from dockerhub. Simply go to the
[opencog](opencog) directory, and run the `docker-build.sh` shell
script. `docker-build.sh -h` for help, and `docker-build.sh -a`
to download everything.

### Obsolete demos
If you have been sent here, and cannot find what you were told about,
look in the master branch of this git repo.

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
