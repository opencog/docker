OpenCog Docker Containers
-------------------------
This directory contains docker configurations for some of OpenCog's
projects.  Some of the notable images include:

* `opencog/atomspace`  -- Core AtomSpace only.
* `opencog/learn`      -- Learning subsystem.
* `opencog/lang-pairs` -- Tabulate and visualize word pairs from
                          text corpora.

Older, obsolete images can be found in the [archive](../archive)
directory. If someone sent you here, and you can't find what you were
told about, then it is probably in the archive.

## Docker setup
1. Install docker. On Debian/Ubuntu, `apt install docker.io`. Be sure to
   ad your userid to the `/etc/group` file, like so:
   `sudo usermod -aG docker $USER`. Log out and log back in, so that
   this new membership can become active.

2. Alternately, follow the
   [docker.com instructions](https://docs.docker.com/engine/installation/)
   for setting up docker. They will want you to create an account. But
   this is not needed. If you do create an account, be sure to say
   `sudo docker login` or

3. Pull images that are used for opencog development by running
   `./docker-build.sh -a`

4. The above pull should have downloaded the latest images. These
   are rebuilt weekly, and thus should be up-to-date, more or less.
   If not, and the absolute very latest is needed, then the docker
   images can be rebuilt. Use `./docker-build.sh -h` to get a list
   of available options.

## Overview
List the current set of docker images with the command `docker images`.
The most notable include:

* `opencog/opencog-deps` -- A base operating system image, with
  most required depdencies installed.

* `opencog/atomspace` -- An image containing the core AtomSpace.
  The AtomSpace is needed by all other subsystems.

* `opencog/learn` -- An image containing the language-learning
  subproject.

The above three images "don't actually do anything"; they just provide
baseline software installs.

Actual demos, which "do actual stuff", are listed below. These need to
be built individually; they cannot be downloaded pre-made. Go to the
relevant directory, and follow the instructions there.

* `opencog/lang-pairs` -- Word-pair counting and visualization. Demo
  of the very first step of the language-learning project.

## Docker Cheat Sheet
* `docker images`
* `docker rmi <image-hex>`
* `docker ps -a`
* `docker create --name foo -it opencog/atomspace`
* `docker start -i <container-name>`
* `docker stop <container-name>`
* `docker rm <container-name>`
* `docker container cp <some-file> <container>:<path>
* `docker attach <container-name>`

## Notes
1. Both `tmux` and `byobu` are installed, so you can use either for
   multiple windows/panes.

## TODO
1. Add more images to github workflow for automated publishing
