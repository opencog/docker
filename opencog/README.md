## Initial setups
The following sub-sections describe the steps required to configure your
workspace

### UNIX Systems
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
3. sudo pip install -U docker-compose # only the first time or when updating
4. Add these lines to .bashrc at $HOME of your PC and restart terminal or run
   `source ~/.bashrc`. __Note that you don't have to clone each repository or
   add all the paths__ , just those you need. For the rest docker-compose will
   create an empty directory.
    * export OPENCOG_SOURCE_DIR=$HOME/path/to/opencog
    * export RELEX_SOURCE_DIR=$HOME/path/to/relex
    * export ATOMSPACE_SOURCE_DIR=$HOME/path/to/atomspace
    * export COGUTILS_SOURCE_DIR=$HOME/path/to/cogutils
    * export MOSES_SOURCE_DIR=$HOME/path/to/moses
    * export OC2MC_SOURCE_DIR=$HOME/path/to/opencog-to-minecraft

### Windows
1. Follow the instruction [here](https://docs.docker.com/engine/installation/windows)
   for setting docker,
2. Start the docker virtual machine using 'Docker Quickstart Terminal' as
   described in the linked web-page from the previous step,
3. Build images using `./docker-build.sh [OPTIONS]`
    * For opencog development use `-bctp` option
    * For NLP related work add `-r` option
    * If you want to update your images add `-u` option. For example for opencog
      development use `-ctu` options. Unless there are some system dependency
      changes, you don't have to update `opeoncog/opencog-deps` image.
    * To list the available options use `-h`
4. In the script `windows-run.sh` found in the same directory as this README,
   Replace '$HOME/path/to/' in the export command to the appropriate absolute
   path on your windows machine. __Note that you don't have to clone each
   repository or add all the paths__, just those you need. Since you will be
   using the MINGW environment that was setup by the first step, use UNIX path
   naming format.

## Steps for OpenCog development
1. Starting the containers
  1. UNIX like systems: run either of the following commands,
    * If you want to map container ports to host run
      `docker-compose run --service-ports dev`
    * If you don't want to map ports to host run
      `docker-compose run dev`
  2. Windows: `./windows-run.sh`

2. For using opencog shells follow instruction
   [here](http://wiki.opencog.org/w/OpenCog_shell)

3. For configuring RelEx in the cogserver run
    * cat /etc/hosts   # take note of the ip address for relex, e.g.
      `172.17.0.69     relex 8e7dc3a09f12 opencog_relex_1`
    * `/tmp/octool -bi` # Build and install opencog
    * start the cogserver, telnet into it and access the scheme shell.
    * `(use-modules (opencog nlp) (opencog nlp chatbot))`
    * `(set! relex-server-host "172.17.0.69")`
    * `(nlp-parse "you know what this is.")`
3. have fun hacking
4. exit container

## Steps for RelEx development (For UNIX like Systems only)
Starting the containers run either of the following commands
* If you want to map container ports to host run
  `docker-compose -f relex.yml run --service-ports relex`
* If you don't want to map ports to host run
   `docker-compose -f relex.yml run relex`

## Steps for opencog-to-minecraft development (For UNIX like Systems only)
1. To start the Minecraft server and access a configured development environment
   run `docker-compose -f minecraft.yml run oc2mc`. Thes server.properties file
   is found in `minecraft/data` in the same folder as this README. The file is
   auto-generated so on changing the entries, restart the server by running
   ` docker restart minecraft-server`.
2. Run `tmux` inside the container for working with multiple windows/panes.
   If you create multiple panes you can use your mouse as an alternative to
   switch between panes.
3. Open a separate terminal, on your host, run `docker attach minecraft-server`.
   This gives you access to the server's console that is used for configuration.
4. Except PYTHONPATH setting step, which isn't needed because it is already
   configured inside the container, follow the steps described
   [here](https://github.com/opencog/opencog-to-minecraft#steps-to-start-the-bot)

## Notes
1. Tmux is preinstalled so you can use it for multiple windows/panes.
2. On exiting opencog container, postgres & relex will still be running in the
   background. So when running `docker-compose run ...` it will auto-link to them,
   provided you haven't removed the containers or shutdown your machine.
2. For more on docker-compose refert to https://docs.docker.com/compose/
