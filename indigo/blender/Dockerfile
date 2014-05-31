# docker build -t $USER/ros-indigo-blender .
# xhost +
# docker run --rm --privileged -i -v /dev/ttyACM0:/dev/ttyACM0 -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -e DISPLAY=:0.0 -t opencog/ros-indigo-blender 

FROM opencog/ros-indigo-dev
MAINTAINER Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get -y update
RUN apt-get -y install blender

RUN apt-get -y install python3-yaml python3-pip
RUN pip3 install rospkg catkin_pkg

WORKDIR /catkin_ws/src
RUN cp /opt/ros/indigo/setup.sh /etc/profile.d/ros_indigo.sh
ENV PYTHONPATH /opt/ros/indigo/lib/python2.7/dist-packages
RUN /usr/bin/python3 /opt/ros/indigo/bin/catkin_init_workspace

RUN git clone https://github.com/hansonrobotics/robo_blender
RUN git clone https://github.com/yantrabuddhi/ros_pololu_servo
RUN git clone https://github.com/yantrabuddhi/ros_nmpt_saliency
RUN git clone https://github.com/yantrabuddhi/ros_faceshift
RUN mkdir ros_faceshift/include

WORKDIR /catkin_ws
RUN bash -l -c "/usr/bin/python3 /opt/ros/indigo/bin/catkin_make"
RUN cp /catkin_ws/devel/setup.bash /etc/profile.d/catkin.sh

WORKDIR /catkin_ws/src/robo_blender/src
CMD /bin/bash -l -c "cd /catkin_ws/src/robo_blender/src && source /opt/ros/indigo/setup.sh && source /catkin_ws/devel/setup.bash && roscore & (sleep 2 && source /catkin_ws/devel/setup.bash && blender robo.blend) && /bin/bash"
