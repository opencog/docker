This directory contains docker configurations for some of OpenCog's projects.
Following are instructions on how to get started with using docker for opencog
development.

# Table of Contents
1. [Initial Setups](#initial-setups)
  1. [UNIX like Systems](#unix-like-systems)
  2. [windows](#windows)
2. [Steps for OpenCog development](#steps-for-opencog-development)
3. [Steps for RelEx development](#steps-for-relex-development)
4. [Steps for opencog-to-minecraft development](#steps-for-opencog-to-minecraft-development)

## Common Initial Setups
The following sub-sections describe the steps required to configure docker on
your os, regardles of which project you are working on.

### UNIX like Systems
1. Follow the instruction [here](https://docs.docker.com/engine/installation/)
   for setting docker,

2. Build images using `./docker-build.sh [OPTIONS]`
    * For opencog development use `-bctp` option
    * For NLP related work use`-r` option
    * For opencog-to-minecraft use `-bcte` option
    * If you want to update your images add `-u` option. For example for opencog
      development use `-ctu` options. Unless there are some system dependency
      changes, you don't have to update `opeoncog/opencog-deps` image.
    * To list the available options use `-h`
3. The following is required only if you want to use docker-compose setup that
   is described below

   ```
   # The following is required only the first time or when updating
   sudo pip install -U docker-compose
   ```

4. Run docker using whichever setup you are accustomed to. If you want to
   use docker-compose, follow the steps below, as per your type of development.
   The docker-compose setup enables you to persist changes you make on the opencog repos, from within the container, to your host OS, even when you
   choose to remove the containers.

### Windows
1. Follow the instruction   
   [here](https://docs.docker.com/engine/installation/windows) for setting
   docker,

2. Start the docker virtual machine using 'Docker Quickstart Terminal' as
   described in the linked web-page from the previous step,

3. Build images using `./docker-build.sh [OPTIONS]`
    * For opencog development use `-bctp` option
    * For NLP related work add `-r` option
    * If you want to update your images add `-u` option. For example for opencog
      development use `-ctu` options. __Unless there are some system dependency
      changes, you don't have to update `opeoncog/opencog-deps` image.__
    * To list the available options use `-h`

4. In the script `windows-run.sh` found in the same directory as this README,
   Replace '$HOME/path/to/' in the export command to the appropriate absolute
   path on your windows machine. __Note that you don't have to clone each
   repository or add all the paths__, just those you need. Since you will be
   using the MINGW environment that was setup by the first step, use UNIX path
   naming format.

5. Run `./windows-run.sh`.

## Steps for OpenCog development
__For UNIX like systems only, and if you choose to use docker-compose__

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
   export OPENCOG_SOURCE_DIR=$HOME/path/to/opencog
   export ATOMSPACE_SOURCE_DIR=$HOME/path/to/atomspace
   export COGUTILS_SOURCE_DIR=$HOME/path/to/cogutils
   export OC2MC_SOURCE_DIR=$HOME/path/to/opencog-to-minecraft
   ```

   __Optionally, if you are using `moses` add to `~/.bashrc`,
   `export MOSES_SOURCE_DIR=$HOME/path/to/moses` and uncomment
   `- $MOSES_SOURCE_DIR:/moses` line in the docker-compose.yml file.__

3. For starting the containers using docker-compose run either of the following
   commands,
  * If you want to map container ports to host run
    `docker-compose run --service-ports dev`
  * If you don't want to map ports to host run
    `docker-compose run dev`

4. For using opencog shells follow instruction
   [here](http://wiki.opencog.org/w/OpenCog_shell)

5. For configuring RelEx in the cogserver run
    * `cat /etc/hosts`   # take note of the ip address for relex, e.g.
      `172.17.0.69     relex 8e7dc3a09f12 opencog_relex_1`
    * `/tmp/octool -bi` # Build and install opencog
    * start the cogserver, telnet into it and access the scheme shell.
    * `(use-modules (opencog nlp) (opencog nlp chatbot) (opencog nlp relex2logic))`
    * `(set-relex-server-host)`
    * `(nlp-parse "you know what this is.")`

6. have fun hacking

7. exit container

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

## Steps for opencog-to-minecraft development
__For UNIX like systems only, and if you choose to use docker-compose__

1. Add these lines to `~/.bashrc` at $HOME of your PC and run
`source ~/.bashrc`.

    ```
    export OPENCOG_SOURCE_DIR=$HOME/path/to/opencog
    export ATOMSPACE_SOURCE_DIR=$HOME/path/to/atomspace
    export COGUTILS_SOURCE_DIR=$HOME/path/to/cogutils
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

## Notes
1. Tmux is preinstalled so you can use it for multiple windows/panes.

2. On exiting opencog container, postgres & relex will still be running in the
   background. So when running `docker-compose run ...` it will auto-link to them,
   provided you haven't removed the containers or shutdown your machine.

3. For more on docker-compose refert to https://docs.docker.com/compose/
