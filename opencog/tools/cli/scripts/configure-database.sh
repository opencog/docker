#!/bin/bash
#
# This is a sample configuration for opencog

psql -U postgres -h db -c "CREATE USER opencog WITH SUPERUSER PASSWORD 'cheese'"
psql -U postgres -h db -c "CREATE DATABASE cogdata OWNER opencog"


# 'cheese' is the password for the next step
cat $HOME/opencog/persist/sql/atom.sql | psql cogdata -U -h db

# For accessing the database run
# psql -h db -d cogdata
