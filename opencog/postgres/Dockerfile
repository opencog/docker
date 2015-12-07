# docker build -t opencog/postgres
# TODO optimize for speeding up atom saves. See https://github.com/opencog/atomspace/tree/master/opencog/persist/sql#performance-tweaks

FROM postgres:latest

ADD https://raw.githubusercontent.com/opencog/atomspace/master/opencog/persist/sql/atom.sql /tmp/atom.sql

COPY configure.sh /docker-entrypoint-initdb.d/
RUN /docker-entrypoint.sh postgres
