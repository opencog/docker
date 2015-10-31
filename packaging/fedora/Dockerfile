FROM fedora:22

RUN dnf install -yv curl sudo
ADD https://raw.githubusercontent.com/opencog/ocpkg/master/install-fedora-dependencies.sh /tmp/setup.sh
RUN chmod u+x /tmp/setup.sh ; /tmp/setup.sh -dcl

# Crete and switch user. The user is privileged with no password required
RUN adduser -c "OpenCog Developer" -G wheel opencog
RUN echo '%wheel ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER opencog
WORKDIR /home/opencog

#Install haskell
RUN sudo chown opencog:opencog /tmp/setup.sh && /tmp/setup.sh -sa

CMD bash
