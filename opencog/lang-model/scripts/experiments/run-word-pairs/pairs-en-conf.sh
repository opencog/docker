#! /bin/bash
#
# Configuration parameters for disjunct distributions.
#
# IPv4 hostname and port number of where the cogserver is running.
export HOSTNAME=localhost
export PORT=19014

export PROMPT="(pairs-en)"

export COGSERVER_CONF=${CONFIG_DIR}/cogserver/cogserver-pairs.conf

export MST_DB=${ROCKS_DATA_DIR}/r14-imp-q0.7-c0.9-n0.rdb
export PAIRS_DB=${ROCKS_DATA_DIR}/run-1-en_pairs-tranche-123.rdb
export ALL_DB=${ROCKS_DATA_DIR}/run-1-t12-tsup-1-1-1.rdb
export ALL_DB=${ROCKS_DATA_DIR}/r13-one-sim200.rdb
export ALL_DB=${ROCKS_DATA_DIR}/r13-all-in-one.rdb

export STORAGE_NODE="(RocksStorageNode \"rocks://${ALL_DB}\")"
