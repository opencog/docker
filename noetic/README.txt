Status of ROS noetic port and bit rot removal
==============================================
This project so far is in the Roadmap 1 stage. See below.

Dependencies
============

eva-ros --> ros-blender --------------------------
                                                  \
                                                   \
eva-silent ----                                     \
               \                                     \
                v                                     v
eva-opencog --> ros-incog-blender --> ros-opencog --> ros-base


Roadmap
=======
1. eva-ros builds
    (will need to replace the vision / face detection subsystem - Done!)
    ISSUES:
    - the "roslaunch robots_config geometry.launch" fails to start
    - blender has issues: NOSE bone missing? dependency cycles ...
                          Sophia has no eyes?
2. eva-ros works
    ISSUES:
    not started
4. eva-silent builds & works
    ISSUES:
    not started
5. eva-opencog builds & works
    ISSUES:
    not started

For Further Reading
===================
See the README.md in ../indigo
