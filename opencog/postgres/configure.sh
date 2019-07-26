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

psql -U postgres -c \
    "CREATE USER opencog_user WITH SUPERUSER PASSWORD '$POSTGRES_PASSWORD'"
psql -U postgres -c "CREATE DATABASE mycogdata OWNER opencog_user"

printf "___ Create the database tables for standard OpenCog use ___\n"
psql -U opencog_user -d mycogdata -a -f /tmp/atom.sql

# For Unit testing
printf "\n___ Configuring for OpenCog unit testing ___\n"
psql -U postgres -c \
    "CREATE USER opencog_tester WITH SUPERUSER PASSWORD '$POSTGRES_PASSWORD'"
psql -U postgres -c "CREATE DATABASE opencog_test OWNER opencog_tester"

## 'cheese' is the password for the next step
printf "\n___ Create the database tables for OpenCog unit testing ___\n"
psql -U opencog_tester -d opencog_test -a -f /tmp/atom.sql

# Continue the running of docker-entrypoint.sh after configuration.
# NOTE: Copy pasted from docker-entrypoint.sh
PGUSER="${PGUSER:-postgres}" pg_ctl -D "$PGDATA" -m fast -w stop

echo
echo 'PostgreSQL init process complete; ready for start up.'
echo

# Exit from process as we don't want to start the server.
exit 0
