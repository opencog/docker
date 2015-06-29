# Usages

You can use docker-compose for configuring your workspace on linux and Mac systems. For Windows details are coming soon.

## Initial setups
1. ./docker-build.sh [OPTIONS]
    * For opencog development use -bt option
    * If you want to update your images add -u option. For example for opencog
      development use -btu
2. sudo pip install -U docker-compose # only the first time
3. Add these lines to .bashrc at $HOME of your PC and restart terminal or run `source ~/.bashrc` . __ Note that you don't have to clone each repository or add
all the paths__ , just those you need. For the rest docker-compose will create
an empty directory.
    * export OPENCOG_SOURCE_DIR=$HOME/path/to/opencog
    * export RELEX_SOURCE_DIR=$HOME/path/to/relex
    * export ATOMSPACE_SOURCE_DIR=$HOME/path/to/atomspace
    * export COGUTIL_SOURCE_DIR=$HOME/path/to/cogutils
    * export MOSES_SOURCE_DIR=$HOME/path/to/moses

## Steps for OpenCog development
1. Starting the contianers run either of the following commands
    * docker-compose run --service-ports dev   # to map container ports to host
    * docker-compose run dev   # if you don't want to map ports to host
2. Configuring postgres backing-store
    * ~/configure-database.sh
    * tmux   # You can create multiple panes and move around using your mouse
    * Start cogserver and enter the scheme shell
    * Follow the steps starting from line 35 in
      $OPENCOG_SOURCE_DIR/examples/guile/persist-example.scm#
3. Configuring RelEx
    * cat /etc/hosts   # take note of the ip address for relex, e.g.
    ```172.17.0.69     relex 8e7dc3a09f12 opencog_relex_1```
    * rlwrap telnet localhost 17001
    * (define relex-server-host "172.17.0.69")
    * (nlp-parse "you know what this is.")
4. have fun hacking
5. exit container

## Notes
1. On exiting opencog container, postgres & relex will still be running in the
   background. So when running `docker-compose run ...` it will autolink to them,
   provided you haven't removed the containers or shutdown your machine.
2. If you just want to work on RelEx the following suffices
   * docker-compose  run --service-ports relex   # to map container ports to host
   * docker-compose run relex   # if you don't want to map ports to hosts
3. For more on docker-compose refert to https://docs.docker.com/compose/
