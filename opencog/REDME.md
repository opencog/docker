# Usages

You can use docker-compose for configuring your workspace on linux systems. For
Mac/Windows details are coming soon.

## Initial setups
1. sudo pip install -U docker-compose # only the first time
2. export OPENCOG_SOURCE_DIR=export OPENCOG_SOURCE_DIR=$HOME/path/to/opencog/

## Steps for OpenCog development
1. docker-compose run dev
2. sudo chown -R opencog:opencog /scripts
2. chmod u+x /scripts/configure-database.sh
3. cp /scripts/odbc.ini $HOME/.odbc.ini
3. /scripts/configure-database.sh   # type cheese on Password prompt
4. have fun hacking
5. exit
6. provided you haven't shutdown your machine or removed the postgres container,
   running step 1 will autolink you with the postgres container which runs in
   background


