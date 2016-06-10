cd $WORKSPACE
chmod g+w .
docker info
docker pull opencog/postgres
docker pull opencog/cogutils
docker images opencog/cogutils
docker inspect --format="{{ .State.Running }}" postgres 2>/dev/null && docker rm -f postgres
docker run -d --name postgres opencog/postgres

exit_status=true
docker run --rm -v $PWD:/atomspace -v /var/opencog/ccache:/home/opencog/.ccache \
  --link postgres:db -e PGHOST=db -e PGUSER=opencog_test \
  -e CCACHE_UMASK=002 -e PATH=/usr/lib/ccache:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  opencog/cogutils /bin/sh -c "umask 002 && cd /atomspace && rm -rf build && /tmp/octool -beti" || exit_status=false

docker inspect --format="{{ .State.Running }}" postgres 2>/dev/null && docker rm -f postgres
$exit_status
