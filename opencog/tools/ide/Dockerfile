# For creating images with gui tools for development, and dependencies installed
# Steps:
# 1. docker build -t opencog/opencog-dev:qtcreator .
# 2. docker create --name opencog -p 17001:17001 -p 18080:18080 -p 5000:5000
#       -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix/:ro -it
#       -w /opencog
#       -it opencog/opencog-dev:qtcreator
# 3. docker start -i opencog
# 4. qtcreator
#  Check http://wiki.opencog.org/w/Using_QT_Creator on how to use qtcreator.

FROM opencog/opencog-dev:cli

# Tools
RUN apt-get -y install git-gui meld qtcreator

# Workspace configuration
COPY qtcreator-startup-script.sh /home/opencog/
RUN chown opencog:opencog /home/opencog/qtcreator-startup-script.sh
USER opencog
CMD /home/opencog/qtcreator-startup-script.sh
