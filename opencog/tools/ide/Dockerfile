# For creating images with gui tools for development, and dependencies installed
# Steps:
# 1. docker build -t opencog/opencog-dev:qtcreator .
# 2. docker create --name opencog -p 17001:17001 -p 5000:5000
#       -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix/:ro -it
#       -w /opencog
#       -it opencog/opencog-dev:qtcreator
# 3. docker start -i opencog
# 4. qtcreator
# Check http://wiki.opencog.org/w/Building_OpenCog#IDEs on how to use IDEs

FROM opencog/opencog-deps
MAINTAINER Alex van der Peet "alex.van.der.peet@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get -y update; apt-get -y upgrade; apt-get -y autoclean

RUN apt-get -y install \
                        git-gui \
                        meld \
                        qtcreator \
                        valgrind

#ENV STARTSCRIPT "\
#echo evaluating startup script... ;\
#cd $HOME;\
#tmux new-session -d 'echo "kernel.yama.ptrace_scope=0" > #/etc/sysctl.d/10-ptrace.conf' ;\
#tmux new-session -d '/usr/bin/gnome-panel&/bin/bash' ;\
#tmux set -g set-remain-on-exit on ;\
#tmux set-option -g set-remain-on-exit on ;\
#tmux bind-key R respawn-window ;\
#tmux split-window -d -v -p 25 '/bin/bash' ;\
#tmux select-layout even-vertical ;\
#tmux attach \
#"

USER opencog

CMD bash
