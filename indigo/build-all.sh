#! /bin/bash
#
# build-all.sh
#
# Handy dandy snippet to build the current demo images.
#
cd ros-base
./build.sh
cd ..

cd ros-opencog
./build.sh
cd ..

cd ros-incog-blender
./build.sh
cd ..

# cd eva-silent
# ./build.sh
# cd ..

cd eva-opencog
./build.sh
cd ..
