#! /bin/bash
#
# count-pairs.sh
#
# Fully-automated word-pair counting, specialized for the current
# Docker setup.
# -----------------

# Everything we need has been copied into run-1.
# Leave it as is, for the user to experiment in.
# Do the actual work in run-2.
# Avoid confusion by removing the config for the later stages.
cd ~/experiments/
cp -pr run-1 run-2
cd run-2
rm -f 3-mpg-conf.sh 4-gram-conf.sh

# Use the default configuration.
source 0-pipeline.sh
source 2-pair-conf.sh

# Remove scripts for later stages (avoid confusion)
cd run
rm -rf 3-mst-parsing 4-gram-class

# Remove semphore, just in case.
rm -f /tmp/pair-marginals-done

# Run the pair-counting pipeline in byobu.
cd 2-word-pairs
./run-all.sh

echo "Done word-pair counting. Good bye!"
