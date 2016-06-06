#!/bin/bash
#
# NOTE:
# 1. This script is used as an entrypoint and is created to make host dependent
#    configurations easier.
# 2. It implements the tweaks suggested @
#    github.com/opencog/atomspace/tree/master/opencog/persist/sql#performance-tweaks

set -e

# This is used to modify the postgres configuration before starting the
# server.
#
# $1 = configuration_parameter
# $2 = value for setting the parameter
alter_pg() {
  echo "Setting $1 configuration parameter to $2"
  psql -U postgres -c "ALTER SYSTEM SET $1 = '$2'"
}

# Returns a value reprsenting the percentage of ram in megabytes.
# $1 = percent of ram. For e.g. 25 for 25%
ram_percent() {
  local RAM_SIZE=$(grep MemTotal /proc/meminfo | awk '{print $2}')
  local RAM_MULTIPLE=$(($RAM_SIZE * $1))
  local RAM_PERCENT="$(($RAM_MULTIPLE / 100000))MB"
  echo "$RAM_PERCENT"
}

if [ "$1" = "postgres" ]; then
  # Perform host specific configurations.

  # Start server
  gosu postgres pg_ctl -D "$PGDATA" \
    -o "-c listen_addresses='localhost'" \
    -w start

  echo "Starting the runtime configurations"
  alter_pg shared_buffers $(ram_percent 25)
  alter_pg work_mem 32MB
  alter_pg effective_cache_size $(ram_percent 50)
  alter_pg fsync off
  alter_pg synchronous_commit off
  # In 9.5 wal_buffers is auto-calcuated as 1/32 of shared_buffers
  alter_pg commit_delay 10000
  # In 9.5 checkpoint_segments doesn't exist
  alter_pg ssl off
  alter_pg autovacuum on
  alter_pg track_counts on

  # Restart for changes to take effect.
  gosu postgres pg_ctl -D "$PGDATA" -m fast -w stop
  gosu postgres postgres
  
else
  exec "$@"
fi
