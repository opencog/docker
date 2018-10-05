#
# Docker file that builds RelEx and starts the RelEx server.
#
# To build:
#    docker build -t relex/relex .
#
# To start:
#    docker run -p 3333:3333 relex/relex /bin/sh plain-text-server.sh
#
# Or alternately, this:
#    docker run -p 4444:4444 relex/relex /bin/sh opencog-server.sh
#
#    docker run -p 9000:9000 relex/relex /bin/sh link-grammar-server.sh
#
# To demo:
#    telnet localhost 4444
#    This is a test sentence!
#
# That is, after connecting by telnet, type in any sentence, ending
# with a period, and hit enter.  The response returned will be the
# parse of the sentence, in opencog scheme format.
#
FROM ubuntu:16.04

# Avoid triggering apt-get dialogs (which may lead to errors). See:
# https://stackoverflow.com/questions/25019183/docker-java7-install-fail
ENV DEBIAN_FRONTEND noninteractive

ENV JAVA_HOME /usr/lib/jvm/java-1.9.0-openjdk-amd64

RUN apt-get update ; apt-get -y upgrade ; apt-get -y autoclean

# Java
RUN apt-get -y install maven screen telnet netcat-openbsd byobu \
                       wget vim git unzip sudo apt-utils

# incorrect packaging in ubuntu-xenial
# dpkg: error processing archive /var/cache/apt/archives/openjdk-9-jdk_9~b114-0ubuntu1_amd64.deb (--unpack):
# trying to overwrite '/usr/lib/jvm/java-9-openjdk-amd64/include/linux/jawt_md.h', which is also in package openjdk-9-jdk-headless:amd64 9~b114-0ubuntu1
RUN apt-get -o Dpkg::Options::="--force-overwrite" -y install openjdk-9-jdk

# GCC and basic build tools
RUN apt-get -y install gcc g++ make

# Wordnet
RUN apt-get -y install wordnet wordnet-dev wordnet-sense-index

# There are UTF8 chars in the Java sources, and the RelEx build will
# break if build in a C environment.
RUN apt-get -y install locales && locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN mkdir /usr/local/share/java

WORKDIR /home/Downloads/

# JWNL - Never changes, so do this first.
RUN wget https://downloads.sourceforge.net/project/jwordnet/jwnl/JWNL%201.4/jwnl14-rc2.zip \
    && unzip jwnl14-rc2.zip; cd jwnl14-rc2; \
    cp jwnl.jar /usr/local/share/java/; \
    chmod 755 /usr/local/share/java/jwnl.jar; \
    cd ..; rm -rf jwnl*

# OpenNLP - Never changes, so do this first.
RUN wget https://archive.apache.org/dist/opennlp/opennlp-1.5.3/apache-opennlp-1.5.3-bin.tar.gz \
    && tar -zxf apache-opennlp-1.5.3-bin.tar.gz ;\
    cd apache-opennlp-1.5.3; \
    cp lib/*.jar /usr/local/share/java/; \
    cp lib/*.jar /usr/share/java/; \
    cp lib/opennlp-tools-1.5.3.jar /usr/local/share/java/opennlp-tools-1.5.0.jar; \
    cd .. ; rm -rf apache-opennlp*

# Link Parser -- changes often
# Download the current released version of link-grammar.
# The wget gets the latest version w/ wildcard
RUN wget -r --no-parent -nH --cut-dirs=2 https://www.abisource.com/downloads/link-grammar/current/
RUN tar -zxf current/link-grammar-5*.tar.gz
# get linkgrammar version
RUN bash -l -c 'echo `ls|grep link|sed 's/link-grammar-//g'` >> LINKGRAMMAR_VERSION'

RUN cd link-grammar-5.*/; ./configure; make -j6; sudo make install; \
    ldconfig;

RUN cd link-grammar-5.*/; mvn install:install-file \
    -Dfile=./bindings/java/linkgrammar-`cat ../LINKGRAMMAR_VERSION`.jar \
    -DgroupId=org.opencog \
    -DartifactId=linkgrammar \
    -Dversion=`cat ../LINKGRAMMAR_VERSION` \
    -Dpackaging=jar

RUN rm -rf * link-grammar*

# Relex -- changes often
ADD https://github.com/opencog/relex/archive/master.tar.gz master.tar.gz
RUN tar -xvf master.tar.gz; cd relex-master; mvn install

# Create and switch user. The user is privileged, with no password
# required.  That is, you can use sudo.
RUN adduser --disabled-password --gecos "ReLex USER" relex
RUN adduser relex sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER relex

RUN sudo chown -R relex:relex .
# Punch out ports
## plain-text-server.sh port
EXPOSE 3333
## opencog-server.sh port
EXPOSE 4444
## link-grammar-server.sh port
EXPOSE 9000

WORKDIR /home/Downloads/relex-master/
CMD /bin/bash
