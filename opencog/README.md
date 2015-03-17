# Usages

You can use docker-compose for configuring your workspace on linux systems. For
Mac/Windows details are coming soon.

## Initial setups
1. ./docker-build.sh
2. sudo pip install -U docker-compose # only the first time
3. export OPENCOG_SOURCE_DIR=$HOME/path/to/opencog/
4. export RELEX_SOURCE_DIR=$HOME/path/to/relex/

## Steps for OpenCog development
1. docker-compose run --service-ports dev   # to map container ports to host
   OR
   docker-compose run dev   # if you don't want to map ports to hosts
2. Configure postgres link
   * sudo chown -R opencog:opencog /scripts
   * chmod u+x /scripts/configure-database.sh
   * cp /scripts/odbc.ini $HOME/.odbc.ini
   * /scripts/configure-database.sh   # type 'cheese' on Password prompt
3. Configure relex link
   * cat /etc/hosts   # take note of the ip address for relex, e.g. 172.17.0.69     relex
   * Start opencog and telnet/netcat into the scheme shell
   * Configure relex-server-host, e.g. (define relex-server-host "172.17.0.69")
   * (nlp-parse "you know what this is.") 
4. have fun hacking
5. exit

## Notes
1. On exiting opencog container, postgres & relex will still be running in the
   background. So when running `docker-compose run ...` it will autolink to them,
   provided you haven't removed the containers or shutdown your machine.
2. If you just want to work on relex
   * docker-compose  run --service-ports relex   # to map container ports to host
   * docker-compose run relex   # if you don't want to map ports to hosts
3. For more on docker-compose refert to https://docs.docker.com/compose/
