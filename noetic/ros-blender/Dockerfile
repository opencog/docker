#
# Docker file for base ROS+Blender install
# No ROS nodes are started.
#
# The primary difficulty here is that Blender wants Python3
# while ROS wants Python2, and so there are install conflicts.
#
# To build:
# docker build -t opencog/ros-noetic-blender .
#
# To run:
# docker run --rm --privileged -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 \
#    -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -e DISPLAY=:0.0 \
#    -t opencog/ros-noetic-blender
#
# The above should be sufficient to get blender running with full
# 3D hardware acceleration on the local host (dri==direct rendering interface)
# Note: /dev/dri=gpu, /dev/shm=gpu
#
FROM opencog/ros-noetic-base
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"
MAINTAINER Linas Vepštas "linasvepstas@gmail.com"

ENV LAST_OS_UPDATE 2020-04-23


# Blender with dependencies
# Install x11-utils to get xdpyinfo, for X11 display debugging
# mesa-utils provides glxinfo, handy for understanding the 3D support.
# Blender needs python3
RUN apt-get -y install  x11-utils mesa-utils python3-yaml python3-pip
###############################################################################
# Blender start
# This Blender block replaces the older indigo method. There may be some dangling redundancies as a
# result (python installs?). The blender package is not installed, just untarred, and a call-in script
# is added at /usr/bin/blender. We do have to munge in the blender libs into the /usr/lib/i386-linux-gnu
# (32 bit) lib. Also: I suppose if there is an easy way to auto-install the tarball we should do it. 
# The issue is that blender works like this because it works relative to where it is started from.
# Otherwise, if it is installed, it seems to get messed up with all the !$%^! python packages.
ENV LAST_OS_UPDATE 2020-04-23_5
# Add i386 support back into ubuntu, it was removed in 20.04 ...
RUN dpkg --add-architecture i386
RUN apt-get -y update
RUN apt-get -y install libc6:i386 libncurses5:i386 libstdc++6:i386 zlib1g:i386

# need some missing blender dependencies
RUN apt-get -y install libxi6:i386 libxrender1:i386 libxxf86vm1:i386 libxfixes3:i386
# fetch blender 2.79b tarball and install it. This is the last Blender to support X11 protocol,
# it migrated to Wayland and direct display rendering with 2.80
RUN mkdir /src /src/blender
ENV BLENDER blender-2.79b-linux-glibc219-i686
ENV I386 /usr/lib/i386-linux-gnu
# fetched the blender tarball so: 
WORKDIR /src/blender
RUN wget "https://download.blender.org/release/Blender2.79/blender-2.79b-linux-glibc219-i686.tar.bz2"
# Un-tar the blender tarball
RUN tar xjf $BLENDER.tar.bz2;
WORKDIR /src/blender/$BLENDER
RUN echo "#!/bin/bash \n\
    /src/blender/$BLENDER/blender " >/usr/bin/blender; \
    chmod +x /usr/bin/blender;
RUN  cp lib/libGLU.so.1.3.08005 $I386/;
RUN  cp lib/libGL.so.1.5.08005 $I386/;
RUN cd $I386; \
    [ -f libGLU.so.1 ] && rm libGLU.so.1; \
    ln -s libGLU.so.1.3.08005 libGLU.so.1; \
    [ -f libGLU.so ] && rm libGLU.so; \
    ln -s libGLU.so.1 libGLU.so; \
    [ -f libGL.so.1 ] && rm libGL.so.1; \
    ln -s libGL.so.1.5.08005 libGL.so.1;
# If you are getting libGL errors make sure the above link(s) are still set, they seem to get messed
# up pretty easily when installing other packages.
# Blender end
################################################################################
# ROS
# We perform some magic incantations to allow ROS (which normally
# expects python2) to happily co-exist with python3.  Yes, its odd
# to do this here, but here is where we first install python3.
RUN pip3 install rospkg catkin_pkg

WORKDIR /catkin_ws/src
RUN cp /opt/ros/noetic/setup.sh /etc/profile.d/ros_noetic.sh
RUN bash -l -c "/usr/bin/python3 /opt/ros/noetic/bin/catkin_init_workspace"

# Must run catkin at least once, even if there is nothing to build;
# it creates the needed directory structure and the needed setup.sh.
WORKDIR /catkin_ws
RUN bash -l -c "/opt/ros/noetic/bin/catkin_make"

# Empty ROS Enviroment Setup
# The user is root; so ~/.bash_profile is /root/.bash_profile
RUN echo source /catkin_ws/devel/setup.bash >> ~/.bash_profile && \
    echo -e "\e[1;34m[$SELF_NAME] catkin devel setup\.bash sourced\e[0m"
# ROS end
#############################################################################

ENV DISPLAY :0.0
CMD /bin/bash
