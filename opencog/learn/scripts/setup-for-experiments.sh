#! /bin/bash
#
# setup-for-experiments.sh
#
# Create a reasonable working environment for running experiments.
# This provides a suggested configuration in which language-learning
# experiments can be run.  Just run this file to do that setup.

mkdir text && mkdir text/input-pages
mkdir data
mkdir experiments && mkdir experiments/run-1
cp -r /usr/local/share/opencog/learn/run-config/*.sh experiments/run-1
cp -r /usr/local/share/opencog/learn/run-common experiments/run-common
chmod ugo+x experiments/run-common/*.sh
chmod ugo+x experiments/run-common/*.pl
cp -r /usr/local/share/opencog/learn/run experiments/run-1
chmod ugo+x experiments/run-1/run/*.sh
chmod ugo+x experiments/run-1/run/*/*.sh
