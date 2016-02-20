#! /bin/bash
#
# build-all.sh
#
# Handy dandy snippet to build all the various images.
#
cd ros-base
./build.sh
cd ..

cd ros-blender
./build.sh
cd ..

cd eva-owyl
./build.sh
cd ..

cd ros-opencog
./build.sh
cd ..

