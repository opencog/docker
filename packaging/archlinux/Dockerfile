FROM pritunl/archlinux

RUN pacman -Sy && pacman  -S --noconfirm sudo
ADD https://raw.githubusercontent.com/opencog/ocpkg/master/install-archlinux-dependencies.sh /tmp/setup.sh

# Not installing b/c atomspace requires python2 while the container has
# python3 . See https://github.com/opencog/atomspace/issues/501
RUN chmod u+x /tmp/setup.sh ; /tmp/setup.sh -dpcl

# Crete and switch user. The user is privileged with no password required
RUN adduser -c "OpenCog Developer" -G wheel opencog
RUN echo '%wheel ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER opencog
WORKDIR /home/opencog

CMD bash
