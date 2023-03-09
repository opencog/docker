#! /bin/bash
#
# count-mst.sh
#
# Fully-automated MST counting, specialized for the current
# Docker setup.
# -----------------

# Fix up ownership of the data files
sudo chown -R opencog:opencog text
sudo chown -R opencog:opencog data

# Everything we need has already been set up in run-1.
# Leave it as-is, in case the user wants to experiment there.
# Do the actual MST counting in run-3.
# Avoid confusion by removing the config for earlier and later stages.
cd ~/experiments/
cp -pr run-1 run-3
cd run-3
rm -f 2-pair-conf.sh 4-gram-conf.sh

# Use the default configuration.
source 0-pipeline.sh
source 3-mpg-conf.sh

# Remove scripts for earlier and later stages (avoid confusion)
cd run
rm -rf 2-word-pairs 4-gram-class

# Remove semaphore, just in case.
rm -f /tmp/mst-marginals-done

# Run the MST counting pipeline in byobu.
cd 3-mst-parsing
./run-all-mst.sh

echo "Done MST counting. Good bye!"
