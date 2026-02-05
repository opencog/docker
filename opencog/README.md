OpenCog Docker Containers
-------------------------
This directory contains docker configurations for some of OpenCog's
projects.  Some of the notable images include:

* `opencog/atomspace`    -- Core AtomSpace only.
* `opencog/atomspace-py` -- AtomSpace plus many Python tools.
* `opencog/learn`        -- Learning subsystem.
* `opencog/lang-pairs`   -- Tabulate and visualize word pairs from
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
   this is not needed. If you do create an account, your probably need
   to say `sudo docker login`.

3. Pull opencog images from dockerhub by running `./docker-build.sh -a`
   These images are automatically rebuilt fairly regularly, and should
   provide reasonably fresh, working code.

4. If the absolute latest is needed, then the docker images can be built
   locally on your machine. The build takes from 20 minutes to an hour,
   depending on your machine and network speeds.
   Use `./docker-build.sh -h` to get a list of available options.

## Overview
List the current set of docker images with the command `docker images`.
The most notable include:

* `opencog/opencog-deps` -- A base operating system image, with
  most required dependencies installed.

* `opencog/atomspace` -- An image containing the core AtomSpace.
  The AtomSpace is needed by all other subsystems.

* `opencog/atomspace-py` -- The AtomSpace, plus a large collection of
  Python tools commonly used for machine learning and DL/NN work.

* `opencog/learn` -- An image containing the language-learning
  subproject.

The above four images don't actually "do anything"; they just provide
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
* `docker create --name foo --hostname bar -it opencog/atomspace`
* `docker start -i <container-name>`
* `docker stop <container-name>`
* `docker rm <container-name>`
* `docker container cp <some-file> <container>:<path>
* `docker attach <container-name>`

## Notes
1. Both `tmux` and `byobu` are installed, so you can use either for
   multiple windows/panes.

## Maintainer Notes
The docker images are built and published every Saturday night, using
a github workflow. However, I think (not sure) that those images are
cached, and thus will contain old code, if the datestamps in the
Dockerfiles aren't bumped. Not sure just right now. The default is
building against Ubuntu 24.04.

## Bonus
There are some "bonus" containers, which might be useful:
* `claude-code` -- Image with AtomSpace plus Anthropic's Claude Code.
* `ollama` -- Image with AtomSpace plus Ollama
