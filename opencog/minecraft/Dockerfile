FROM opencog/opencog-dev:cli

# Install ros
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
RUN apt-get update && apt-get -y install ros-indigo-ros-base

# Spock setup
RUN easy_install pip ; apt-get -y install python3-dev python3-setuptools ;\
   pip install cryptography
RUN wget -P /tmp/  https://github.com/SpockBotMC/SpockBot/archive/master.tar.gz ;\
    cd /tmp ; tar -xvf master.tar.gz ; cd SpockBot-master/ ;\
    python3 setup.py install
    # ; cd .. ; rm -rf master.tar.gz SpockBot-master

USER opencog
