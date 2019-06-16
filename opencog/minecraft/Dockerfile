FROM opencog/opencog-dev:cli

# ROS installation
# https://github.com/osrf/docker_images/blob/master/ros/indigo/indigo-ros-core/
# https://github.com/osrf/docker_images/blob/master/ros/indigo/indigo-ros-base/
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV ROS_DISTRO kinetic

##  python3-dev & python3-setuptools are for spock included in this
## instruction for minimizing layers
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    ros-kinetic-ros-base \
    python3-dev python3-setuptools \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init

COPY ./ros_entrypoint.sh /

# Spock installation
RUN easy_install pip &&  pip install cryptography
RUN wget -P /tmp/ https://github.com/SpockBotMC/SpockBot/archive/v0.1.5.tar.gz ;\
    cd /tmp ; tar -xvf v0.1.5.tar.gz ; cd SpockBot-0.1.5/ ;\
    python setup.py install ;\
    cd .. ; rm -rf v0.1.5.master.tar.gz SpockBot-0.1.5

# Installing tmuxinator
## This is NOT the "correct" way of getting this to work. Really should be done the the debian way.
# RUN apt-get update && apt-get install ruby -y;\
RUN gpg --keyserver hkp://pool.sks-keyservers.net --recv-keys 409B6B1796C275462A1703113804BB82D39DC0E3 7D2BAF1CF37B13E2069D6956105BD0E739499BDB
RUN wget -qO- https://get.rvm.io | bash -s stable
#RUN cat /etc/profile.d/rvm.sh | bash
RUN /usr/local/rvm/bin/rvm requirements
RUN /usr/local/rvm/bin/rvm install 2.4.6
RUN /bin/bash -l -c "rvm use 2.4.6 --default"
RUN /bin/bash -l -c "gem install tmuxinator"

USER opencog
RUN  rosdep update
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
