#! /bin/bash
#
# count-pairs.sh
#
# Fully-automated word-pair counting, specialized for the current
# Docker setup.
# -----------------

# Everything we need has been copied into run-1.
# Remove the config for the later stages.
cd ~/experiments/run-1/
rm -f 3-mpg-conf.sh 4-gram-conf.sh

# Use the default configuration.
source 0-pipeline.sh
source 2-pair-conf.sh

# Remove scripts for later stages (avoid confusion)
cd run
rm -rf 3-mst-parsing 4-gram-class

# Run the the pair-counting pipeline in the byobu.
cd 2-word-pairs
./run-all.sh

echo "Done counting. Good bye!"
