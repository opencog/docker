#!/bin/bash
#
# This is a sample configuration for opencog that is to be run within
# the opencog container(i.e. the container that runs opencog). The
# environment varaible OPENCOG_SOURCE_DIR should be set for this to
# work; if you are using docker-compose, it is set for you.

psql -U postgres -h db -c "CREATE USER opencog WITH SUPERUSER PASSWORD 'cheese'"
psql -U postgres -h db -c "CREATE DATABASE cogdata OWNER opencog"

# 'cheese' is the password for the next step
cat $OPENCOG_SOURCE_DIR/opencog/persist/sql/atom.sql | psql cogdata -U opencog -h db

# For accessing the database run
# psql -h db -d cogdata
