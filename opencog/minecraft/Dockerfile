FROM ros:latest

# http://www.webupd8.org/2014/03/how-to-install-oracle-java-8-in-debian.html
# https://github.com/airdock-io/docker-oracle-jdk/blob/master/oracle-java8/Dockerfile
RUN echo "deb http://ppa.launchpad.net/webupd8team/java/ubuntu \
    $(lsb_release -cs) main" > /etc/apt/sources.list.d/webupd8team-java.list ;\
    echo "deb-src http://ppa.launchpad.net/webupd8team/java/ubuntu \
    $(lsb_release -cs) main" >> /etc/apt/sources.list.d/webupd8team-java.list ;\
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys EEA14886 ;\
    apt-get update ; \
    echo "oracle-java8-installer shared/accepted-oracle-license-v1-1 \
    select true" |  /usr/bin/debconf-set-selections ;\
    apt-get -y install oracle-java8-installer oracle-java8-set-default

# Create and switch user. The user is privileged with no password required
RUN adduser --disabled-password --gecos "OpenCog Developer" opencog
RUN adduser opencog sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER opencog
WORKDIR /home/opencog
