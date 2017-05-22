#!/bin/bash

cp ../base/scripts/odbc-localhost.ini ~/.odbc.ini

STARTED="FALSE"

for i in $(docker ps -a --format={{.Names}}); do
    if [ $i == "opencog-postgres" ]; then
        echo "Starting container opencog-postgres"
        docker start opencog-postgres
        STARTED="TRUE"
        echo "Started container opencog-postgres"
    fi
done

if [ $STARTED == "FALSE" ]; then
    echo "Starting container opencog-postgres"
    docker run  -d --name opencog-postgres -e PROD="True" -p 5432:5432 opencog/postgres
    echo "Started container opencog-postgres"
fi
