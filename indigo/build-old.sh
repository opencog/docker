#! /bin/bash
#
# build-old.sh
#
# Handy dandy snippet to build the old images.
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
