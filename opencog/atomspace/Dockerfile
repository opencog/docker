#
# Container with just the core AtomSpace framework installed.
# This includes the cogserver, atomspce-rocks and atomspace-cog.
# It does NOT include nlp, unify, ure, pln or opencog.
#
# Steps:
# 1. docker build --no-cache -t opencog/atomspace .
# 2. docker create --name atomspace -p 17001:17001 -p 18080:18080
#       -v /ABSOLUTE/PATH/TO/YOUR/WORKING/DIRECTORY/On/Your/PC/:/opencog
#       -w /opencog
#       -it opencog/atomspace
# 3. docker start -i atomspace
#

ARG OS_VERSION=latest
ARG BASE_IMAGE=opencog/opencog-deps:${OS_VERSION}
FROM ${BASE_IMAGE}

# The apt installs below will fail, unless we are at the latest.
# Bump the date, if there are DNS failures!
ENV LAST_APT_UPDATE=2025-08-08
RUN apt-get -y update; apt-get -y upgrade; apt-get -y autoremove

# Basic tools that allow this image to be minimally usable.
#  * Networking tools (ping, ifconfig, telnet, netcat)
#  * Terminal sharing tools (tmux, byobu)
#  * Debug tools (gdb, xxd, jq)
#  * Webserver (apache2, php)
#
# Oddly, the xxd package is sometimes missing; use --ignore-missing.
# The || true is needed to prevent docker from dying when xxd is not
# found.
RUN apt-get -y --ignore-missing install \
	lsof \
	net-tools iputils-ping netcat-openbsd traceroute iptraf \
	rlwrap telnet \
	tmux byobu \
	gdb time xxd jq \
	apache2 php \
	|| true

# Install cogutil
ENV LAST_COGUTIL_UPDATE=2025-11-17
RUN  /tmp/octool -c && ccache -C

# Install core AtomSpace.
ENV LAST_ATOMSPACE_UPDATE=2025-11-20
RUN  /tmp/octool -a && ccache -C

USER opencog
WORKDIR /home/opencog

# Setup the workspace for developers
COPY /scripts/ /home/opencog
RUN sudo chown -R opencog:opencog .
RUN sudo chown -R opencog:opencog .[a-z]*

# XXX FIXME - despite efforts, the timestamp on
#    /usr/local/lib/guile/3.0/site-ccache/opencog.go
# is older than that on
#    /usr/local/share/guile/site/3.0/opencog.scm
# So hack around this by forcing a compile.
RUN guile -c "(use-modules (opencog) (opencog exec) (opencog cogserver) (opencog persist) (opencog persist-cog) (opencog persist-rocks))"

# RUN guile -c "(use-modules (opencog persist-pgres))"

# Docker EXPOSE says "we listen on these ports". Which is not quite
# true, because the default cogserver publishes on these ports. But
# it is considered to be "best practices" to announce this to the world.
EXPOSE 17001
EXPOSE 18080
EXPOSE 18888

CMD /bin/bash

ONBUILD USER root
