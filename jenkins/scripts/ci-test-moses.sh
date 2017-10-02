cd $WORKSPACE
chmod g+w .

docker info
docker pull opencog/cogutil
docker images opencog/cogutil

docker run --rm -v $PWD:/moses -v /var/opencog/ccache:/home/opencog/.ccache \
  -e CCACHE_UMASK=002 -e PATH=/usr/lib/ccache:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  opencog/cogutil /bin/sh -c "cd /moses && rm -rf build && /tmp/octool -beti"
