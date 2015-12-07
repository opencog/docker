# Usages

You can use docker-compose for configuring your workspace on linux and Mac systems. For Windows details are coming soon.

## Initial setups
1. ./docker-build.sh [OPTIONS]
    * For opencog development use -bct option
    * If you want to update your images add `-u` option. For example for opencog
      development use `-ctu` options. Unless there are some system dependency changes, you don't have to update `opeoncog/opencog-deps` image.
2. sudo pip install -U docker-compose # only the first time
3. Add these lines to .bashrc at $HOME of your PC and restart terminal or run `source ~/.bashrc` . __ Note that you don't have to clone each repository or add
all the paths__ , just those you need. For the rest docker-compose will create
an empty directory.
    * export OPENCOG_SOURCE_DIR=$HOME/path/to/opencog
    * export RELEX_SOURCE_DIR=$HOME/path/to/relex
    * export ATOMSPACE_SOURCE_DIR=$HOME/path/to/atomspace
    * export COGUTILS_SOURCE_DIR=$HOME/path/to/cogutils
    * export MOSES_SOURCE_DIR=$HOME/path/to/moses

## Steps for OpenCog development
1. Starting the containers run either of the following commands
    * `docker-compose run --service-ports dev`   # to map container ports to host
    * `docker-compose run dev`   # if you don't want to map ports to host
2. Configuring RelEx
    * cat /etc/hosts   # take note of the ip address for relex, e.g.
    ```172.17.0.69     relex 8e7dc3a09f12 opencog_relex_1```
    * rlwrap telnet localhost 17001
    * (define relex-server-host "172.17.0.69")
    * (nlp-parse "you know what this is.")
3. have fun hacking
4. exit container

## Steps for RelEx development
Starting the containers run either of the following commands
* `docker-compose -f relex.yml run --service-ports relex`  # to map container ports to host
* `docker-compose -f relex.yml run relex`  # if you don't want to map ports to host

## Notes
1. On exiting opencog container, postgres & relex will still be running in the
   background. So when running `docker-compose run ...` it will auto-link to them,
   provided you haven't removed the containers or shutdown your machine.
2. For more on docker-compose refert to https://docs.docker.com/compose/
