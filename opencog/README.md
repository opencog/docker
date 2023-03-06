OpenCog Docker Containers
-------------------------
This directory contains docker configurations for some of OpenCog's
projects.  The instructions below explain how to get started with using
docker to deploy production servers, as well as for development.
Some of the notable containers include:

* `opencog/atomspace`  -- Core AtomSpace only.
* `opencog/learn`      -- Learning subsystem.
* `opencog/lang-pairs` -- Tabulate and visualize word pairs from
                          text corpora.

Older, obsolete containers can be found in the [archive](../archive)
directory. If someone sent you here, and you can't find what you were
told about, then it is probably in the archive.

## Table of Contents
1. [Common Initial Setup](#common-initial-setup)
   1. [Linux and UNIX-like Systems](#linux-and-unix-like-systems)
   2. [Windows](#windows)
2. [Running Production Servers](#running-production-servers)
3. [Steps for OpenCog development](#steps-for-opencog-development)

## Common Initial Setup
The following sub-sections describe the steps required to configure docker on
your OS, regardless of which project you are working on.

### Linux and UNIX-like Systems
1. Follow the instructions [here](https://docs.docker.com/engine/installation/)
   for setting up docker.

2. Pull images that are used for opencog development by running
   `./docker-build.sh -a`
3. Build images using `./docker-build.sh [OPTIONS]`.
    * For language learning work, use the `-l` option.
    * If you want to update your images add `-u` option. For example,
      for opencog development, use `-ctu` options. Unless there are
      some base OS changes, or changes to ocpkg/octool, you don't have
      to update `opencog/opencog-deps` image.
    * To list the available options, use `-h`


## Running Production Servers
Several different server containers are provided. The most important of
these are:
* The atomspace container, which contains the core atomspace that
  everyone needs. It does not contain any of the more experimental
  repos, nor any of the older, deprecated or abandoned projects.
* The development container, which contains everything, including
  development tools, and most (but not all!) of the experimental
  repos.

## Docker Cheat Sheet
* `docker images`
* `docker rmi <image-hex>`
* `docker ps -a`
* `docker rm <image-name>`

## Notes
1. Both `tmux` and `byobu` are installed, so you can use either for
   multiple windows/panes.

## TODO
1. Add more images to github workflow for automated publishing
