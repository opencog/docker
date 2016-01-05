FROM opencog/opencog-dev:cli

# ROS installation
# https://github.com/osrf/docker_images/blob/master/ros/indigo/indigo-ros-core/
# https://github.com/osrf/docker_images/blob/master/ros/indigo/indigo-ros-base/
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV ROS_DISTRO indigo

RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

##  python3-dev & python3-setuptools are for spock included in this
## instruction for minimizing layers
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    ros-indigo-ros-base \
    python3-dev python3-setuptools \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init

COPY ./ros_entrypoint.sh /

# Spock installation
RUN easy_install pip &&  pip install cryptography
RUN wget -P /tmp/ https://github.com/SpockBotMC/SpockBot/archive/master.tar.gz ;\
    cd /tmp ; tar -xvf master.tar.gz ; cd SpockBot-master/ ;\
    python3 setup.py install ;\
    cd .. ; rm -rf master.tar.gz SpockBot-master

USER opencog
RUN  rosdep update
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
