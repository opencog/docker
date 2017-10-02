cd $WORKSPACE
chmod g+w .
docker info
docker pull opencog/opencog-deps
docker images opencog/opencog-deps

docker run --rm -v $PWD:/cogutil -v /var/opencog/ccache:/home/opencog/.ccache \
  -e CCACHE_UMASK=002 -e PATH=/usr/lib/ccache:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  opencog/opencog-deps /bin/sh -c "umask=002 && cd /cogutil && rm -rf build && /tmp/octool -bti"
