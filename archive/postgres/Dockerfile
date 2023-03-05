# docker build -t opencog/postgres

FROM postgres:9.5

# TODO:  Change to build time argument. See https://github.com/docker/docker/issues/13490
ENV POSTGRES_PASSWORD="cheese"

ADD https://raw.githubusercontent.com/opencog/atomspace/master/opencog/persist/sql/multi-driver/atom.sql /tmp/atom.sql

COPY configure.sh /docker-entrypoint-initdb.d/
COPY start.sh /

RUN chown $(id -u postgres):$(id -g postgres) /tmp/atom.sql \
    /docker-entrypoint-initdb.d/configure.sh

# Setup the databases for OpenCog use
ENV PGDATA=/data
RUN /docker-entrypoint.sh postgres

# Start postgres on default  without going through the configuration steps.
ENTRYPOINT ["/start.sh"]
CMD ["postgres"]
