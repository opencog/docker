#! /bin/bash
#
# build-all.sh
#
# Handy dandy snippet to build all the various images.
#
cd base
./build.sh
cd ..

cd blender
./build.sh
cd ..

cd eva
./build.sh
cd ..
