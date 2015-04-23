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
psql -U postgres -c "CREATE USER opencog_user WITH SUPERUSER PASSWORD 'cheese'"

psql -U postgres -c "CREATE DATABASE mycogdata OWNER opencog_user"

## 'cheese' is the password for the next step
cat $OPENCOG_SOURCE_DIR/opencog/persist/sql/atom.sql | psql mycogdata -U opencog_user

# For Unit testing
psql -U postgres -c "CREATE USER opencog_tester WITH SUPERUSER PASSWORD 'cheese'"

psql -U postgres -c "CREATE DATABASE opencog_test OWNER opencog_tester"

## 'cheese' is the password for the next step
cat $OPENCOG_SOURCE_DIR/opencog/persist/sql/atom.sql | psql opencog_test -U opencog_tester
