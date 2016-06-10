cd $WORKSPACE
chmod g+w .

docker info
docker pull opencog/cogutils
docker images opencog/cogutils

docker run --rm -v $PWD:/moses -v /var/opencog/ccache:/home/opencog/.ccache \
  -e CCACHE_UMASK=002 -e PATH=/usr/lib/ccache:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  opencog/cogutils /bin/sh -c "cd /moses && rm -rf build && /tmp/octool -beti"
