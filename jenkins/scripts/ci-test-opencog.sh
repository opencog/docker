cd $WORKSPACE
chmod g+w .
docker info
docker pull opencog/opencog-dev:cli
docker images opencog/opencog-dev:cli

docker run --rm -v $PWD:/opencog -v /var/opencog/ccache:/home/opencog/.ccache \
  -e PYTHONPATH=/usr/local/share/opencog/python:/opencog/opencog/python/:/opencog/build/opencog/cython:/opencog/opencog/nlp/anaphora \
  -e CCACHE_UMASK=002 -e PATH=/usr/lib/ccache:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  opencog/opencog-dev:cli /bin/sh -c "cd /opencog && rm -rf build && /tmp/octool -beti"
