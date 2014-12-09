#
# build-all.sh
#
# Handy dandy snippet to build all the various images.
#
docker build -t opencog/ros-indigo-base .

cd blender
docker build -t opencog/ros-indigo-blender .

cd ..

