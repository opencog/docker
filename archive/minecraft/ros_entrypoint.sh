#!/bin/bash
# TODO: 1. Move this script to the base class
#       2. add new env variables for modifying PYTHONPATH. That way the export
#          below wouldn't be built to images.

set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# For oc2mc
export PYTHONPATH=$PYTHONPATH:/oc2mc:/oc2mc/minecraft_bot/src
exec "$@"
