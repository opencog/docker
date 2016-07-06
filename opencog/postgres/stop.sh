#!/bin/bash

set -e
echo "Stopping container opencog-postgres"
docker stop opencog-postgres
echo "Stopped container opencog-postgres"
