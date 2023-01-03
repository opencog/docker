#
# Container with core learning repos. No data.
#
# Steps:
# 1. docker build --no-cache -t opencog/learn .
# 2. docker create --name learn -p 17001:17001 -it opencog/learn
#    Note: the -p flag is external_port:internal_port
# 3. docker start -i learn
# 4. Copy your text files to /home/opencog/text/input-pages
# 5. cd /home/opencog/experiments/run-1
# 6. Review the config files; change as desired.
# 7. Run run-tmux.sh

FROM opencog/atomspace

RUN  /tmp/octool -z && ccache -C

USER opencog
WORKDIR /home/opencog

RUN mkdir /home/opencog/data
RUN mkdir /home/opencog/text
RUN mkdir /home/opencog/text/input-pages
RUN mkdir /home/opencog/experiments
RUN mkdir /home/opencog/experiments/run-1
RUN cp -r /usr/local/share/opencog/learn/run-config/*.sh /home/opencog/experiments/run-1
RUN cp -r /usr/local/share/opencog/learn/run /home/opencog/experiments/run-1

CMD /bin/bash

ONBUILD USER root
