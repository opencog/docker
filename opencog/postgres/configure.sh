#!/bin/bash
#
# This is a configuration script for opencog that is to be run within
# the opencog container(i.e. the container that runs opencog). The
# environment varaible OPENCOG_SOURCE_DIR should be set for this to
# work; if you are using docker-compose, it is set for you. You might
# have to wait for a few seconds until the postgres starts before
# running the script. Once run you are set.

# For Standard Use
## After setup for accessing the database run
## psql -h db -U opencog_user -d mycogdata
printf "___ Configuring for standard OpenCog use ___\n"

psql -U postgres -c "CREATE USER opencog_user WITH SUPERUSER PASSWORD 'cheese'"
psql -U postgres -c "CREATE DATABASE mycogdata OWNER opencog_user"

## 'cheese' is the password for the next step
cat /tmp/atom.sql | psql mycogdata -U opencog_user

# For Unit testing
printf "\n___ Configuring for OpenCog unit testing ___\n"

psql -U postgres -c "CREATE USER opencog_tester WITH SUPERUSER PASSWORD 'cheese'"
psql -U postgres -c "CREATE DATABASE opencog_test OWNER opencog_tester"

## 'cheese' is the password for the next step
cat /tmp/atom.sql | psql opencog_test -U opencog_tester

# Prevent the run of this script on startup. Since /docker-entrypoint.sh
# in postgres image will execute any *.sh file on docker run and this script
# is already run on image build.
# NOTE: docker-entrypoint.sh runs this script in the same process thus
# the environment variables had to be named explicitly, see
# http://stackoverflow.com/a/8352939/2322857
FILE_NAME="configure.sh"
DIR_PATH="docker-entrypoint-initdb.d"
mv "$DIR_PATH/$FILE_NAME" "$DIR_PATH/$FILE_NAME.done"

# Continue the running of docker-entrypoint.sh after configuration.
# NOTE: Copy pasted from docker-entrypoint.sh
gosu postgres pg_ctl -D "$PGDATA" -m fast -w stop
set_listen_addresses '*'

echo
echo 'PostgreSQL init process complete; ready for start up.'
echo

# Exit from process as we don't want to start the server.
exit 0
