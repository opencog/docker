Obsolete OpenCog Docker Images
------------------------------
This directory contains all of the unmaintained, abandoned and
obsolete docker configurations for some of OpenCog's older projects.
The instructions below are a snapshot of the instructions, as they
used to be, circa 2015, and so may feel anachronistic. The goal was
to explain... *How to get started with using docker to deploy
production servers, as well as for development.* ... and so on.

Some of the notable containers include:

* `opencog/cogserver` -- An empty CogServer container.
* `opencog/opencog-dev` -- All supported OpenCog components.

* `opencog/opencog-dev:cli`: This depends on the
  `opencog/atomspace:latest` image. It installs all supported opencog
  repos and tools, including `as-moses`.

* `opencog/opencog-dev:ide`: To be used for developing using IDEs.
   QtCreator is installed.

* `opencog/opencog-jupyter`: Above, plus a Jupyter notebook.

All of the containers above are "empty", in that they contain no data,
and thus, they don't "do anything".

## Table of Contents
1. [Common Initial Setup](#common-initial-setup)
   1. [Linux and UNIX-like Systems](#linux-and-unix-like-systems)
   2. [Windows](#windows)
2. [Running Production Servers](#running-production-servers)
3. [Steps for OpenCog development](#steps-for-opencog-development)
4. [Steps for RelEx development](#steps-for-relex-development)
RelEx is deprecated/obsolete.
5. [Steps for opencog-to-minecraft development](#steps-for-opencog-to-minecraft-development)
Minecraft is obsolete.

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
    * For opencog development use `-bct` option
    * For NLP related work use `-r` option
    * For opencog-to-minecraft use `-bcte` option

4. The following is required only if you want to use `docker-compose`
   setup that is described below. You only need to do this once, or
   repeat it when updating.

   ```
   sudo apt install docker-compose
   ```

5. Run docker using whichever setup you are accustomed to. If you want
   to use `docker-compose`, follow the steps below, as per your type
   of development.  The `docker-compose` setup enables you to persist
   changes you make on the opencog repos, from within the container,
   to your host OS, even when you choose to remove the containers.

### Windows
1. Follow the instruction
   [here](https://docs.docker.com/engine/installation/windows) for setting
   docker,

2. Start the docker virtual machine using 'Docker Quickstart Terminal' as
   described in the linked web-page from the previous step,

3. Pull images that are used for opencog development by running
   `./docker-build.sh -a`
4. Build images using `./docker-build.sh [OPTIONS]`
    * If you want to update your images add `-u` option. For example,
      for opencog development, use `-ctu` options. Unless there are
      some base OS changes, or changes to ocpkg/octool, you don't have
      to update `opencog/opencog-deps` image.
    * To list the available options, use `-h`
    * For opencog development use `-bct` option
    * For NLP related work add `-r` option

5. In the script `windows-run.sh` found in the same directory as this README,
   Replace `$HOME/path/to/` in the export command to the appropriate absolute
   path on your windows machine. __Note that you don't have to clone each
   repository or add all the paths__, just those you need. Since you will be
   using the MINGW environment that was setup by the first step, use UNIX path
   naming format.

6. Run `./windows-run.sh`.

## Running Production Servers
Several different server containers are provided. The most important of
these are:
* The atomspace container, which contains the core atomspace that
  everyone needs. It does not contain any of the more experimental
  repos, nor any of the older, deprecated or abandoned projects.
* The development container, which contains everything, including
  development tools, and most (but not all!) of the experimental
  repos, and some of the deprecated repos that are still in use.

## Steps for OpenCog development
__For UNIX-like systems only, if you choose to use docker-compose__

1. Specify a directory for storing ccache's compiler outputs on your host OS

   ```
   mkdir -p $HOME/path/to/where/you/want/to/save/ccache/output
   export CCACHE_DIR=$HOME/path/to/where/you/want/to/save/ccache/output`
   ```

   Specifying this means, making a clean build of source code after removing a
   container will be faster. If you don't want to do this, then comment out the
   `- $CCACHE_DIR:/home/opencog/.ccache` in `common.yml` file

2. Add these lines to `~/.bashrc` at $HOME of your host OS and run
   `source ~/.bashrc`.

   ```
   export ETC_LOCALTIME=$(readlink /etc/localtime)
   export OPENCOG_SOURCE_DIR=$HOME/path/to/opencog
   export ATOMSPACE_SOURCE_DIR=$HOME/path/to/atomspace
   export COGUTIL_SOURCE_DIR=$HOME/path/to/cogutil
   export OC2MC_SOURCE_DIR=$HOME/path/to/opencog-to-minecraft
   ```

   Optionally, if you are using `moses` add to `~/.bashrc`,
   `export MOSES_SOURCE_DIR=$HOME/path/to/moses` and uncomment
   `- $MOSES_SOURCE_DIR:/moses` line in the docker-compose.yml file.

3. For starting the containers using docker-compose run either of the following
   commands,
    * If you want to map container ports to host run
      `docker-compose run --service-ports dev`
    * If you don't want to map ports to host run
      `docker-compose run dev`

4. The container has opencog installed, but if you want to install
   cogutil, atomspace, or opencog due to some change you made in
   the source code found on your host, `cd` into the mount directory inside
   the container and then run `/tmp/octool -bi`

5. For using opencog shells follow instruction
   [here](http://wiki.opencog.org/w/OpenCog_shell)

6. Have fun hacking

7. Exit container

## Steps for RelEx development
RelEx is deprecated/obsolete. It builds and works just fine, and is
perfectly usable. However, it is no longer maintained, and is not
recommended for any future development.  See the (commented out)
instructions in this README for details.

RelEx is deprecated/obsolete.

6. For configuring RelEx in the cogserver run
    * start the cogserver, telnet into it and access the scheme shell.
    * `(use-modules (opencog nlp) (opencog nlp chatbot) (opencog nlp relex2logic))`
    * `(set-relex-server-host)`
    * `(nlp-parse "you know what this is.")`

## Steps for RelEx development
__For UNIX like systems only, and if you choose to use docker-compose__

1. Add these lines to `~/.bashrc` at $HOME of your PC and run
   `source ~/.bashrc`.

   ```
   export RELEX_SOURCE_DIR=$HOME/path/to/relex
   ```

2. For starting the containers run either of the following commands,
  * If you want to map container ports to host run
    `docker-compose -f relex.yml run --service-ports relex`
  * If you don't want to map ports to host run
     `docker-compose -f relex.yml run relex`

## Steps for running jupyter kernels with opencog
For some reason, the Jupyter container no longer builds. There is some
bug with setting up the python environment. If you know how to use
python, please fix this (these) config bugs (and remove this error
message when done.)

1. Build the image which has python and guile kernels installed with the following command.
    ```
    ./docker-build -j
    ```
This would build/pull necessary docker images.

2. Add this lines to `./.bashrc` to point to your preferred notebooks save directory

    ```
    export OPENCOG_NOTEBOOKS=$HOME/path/to/opencog_notebooks
    ```
3. For starting jupyter notebook run the following command.

    ```
    docker-compose -f opencog-jupyter.yml run --service-ports notes
    ```
4. Go to `0.0.0.0:8888/tree/notebooks` to interact with your notebooks.

## Steps for opencog-to-minecraft development
The minecraft code is currently bit-rotted, and will not build.
See the (commented out) instructions in this README for details.

## Steps for opencog-to-minecraft development
__For UNIX like systems only, and if you choose to use docker-compose__

1. Add these lines to `~/.bashrc` at $HOME of your PC and run
`source ~/.bashrc`.

    ```
    export ETC_LOCALTIME=$(readlink /etc/localtime)
    export OPENCOG_SOURCE_DIR=$HOME/path/to/opencog
    export ATOMSPACE_SOURCE_DIR=$HOME/path/to/atomspace
    export COGUTIL_SOURCE_DIR=$HOME/path/to/cogutil
    export OC2MC_SOURCE_DIR=$HOME/path/to/opencog-to-minecraft
    ```

2. To start the Minecraft server and access a configured development environment
   run `docker-compose -f minecraft.yml run oc2mc`. The server.properties file
   is found in `minecraft/data` in the same folder as this README. The file is
   auto-generated so on changing the entries, restart the server by running
   ` docker restart minecraft-server`.

3. Run `tmux` inside the container for working with multiple windows/panes.
   If you create multiple panes you can use your mouse as an alternative to
   switch between panes.

4. Open a separate terminal, on your host, run `docker attach minecraft-server`.
   This gives you access to the server's console that is used for configuration.

5. Except PYTHONPATH setting step, which isn't needed because it is already
   configured inside the container, follow the steps described
   [here](https://github.com/opencog/opencog-to-minecraft#steps-to-start-the-bot)

## Steps for opencog perception development
The perception code is currently bit-rotted, and will not build.
See the (commented out) instructions in this README for details.

## Steps for opencog perception development
__WIP and only for use with systems with gpus, for now__

1. Install nvidia docker plugin by following instruction
   [here](https://github.com/NVIDIA/nvidia-docker/wiki/Installation).

2. Build openog/perception image by running
   `docker build -t opencog/perception perception/` from this directory.

3. For usage of the built image see
   [here](https://github.com/NVIDIA/nvidia-docker/wiki/nvidia-docker).

## Docker Cheat Sheet
* `docker images`
* `docker rmi <image-hex>`
* `docker ps -a`
* `docker rm <image-name>`

## Notes
1. Both `tmux` and `byobu` are installed, so you can use either for
   multiple windows/panes.

2. On exiting the opencog container, postgres & relex will still be
   running in the background. So when running `docker-compose run ...`
   it will auto-link to them, provided you haven't removed the
   containers or shutdown your machine.

3. For more on docker-compose refer to https://docs.docker.com/compose/

## TODO
1. Update docker-compose configuration for minimizing steps to start
   developing

2. Add more images to github workflow for automated publishing
