lang-mst
--------
Compute Maximum Spanning Tree (MST) disjuncts from text.

This container provides a basic demo, together with a fully automated
script, for counting MST disjuncts in text. It illustrates the second
stage of the language-learning pipeline. It assumes that the first
stage, from the `lang-pairs` container, has been done, and the resulting
dataset is available.

New users should follow the manual process, given below. Instructions
for a fully-automated run are given at the end.

Common setup
------------
Setup needed for both the manual demo, and the full-automated runs.

1. Create this Docker image:
```
     docker build -t opencog/lang-mst .
```
2. Create a container:
   `docker create --name mst-counter -it opencog/lang-mst`

3. (Optional) If you want access to the CogServer from outside the
   container, be sure to export the CogServer port:
   `docker create --name mst-counter -p 17003:17003 -it opencog/lang-mst`
   Note that this is a different port, from the pair-counting port;
   thus, you can run both at the same time.

4. Copy the word-pair database into the container. The default config
   files expect it at `data/mpg_parse.rdb`. The `lang-pairs` docker
   scripts previously dropped off the pairs DB at `data/word_pairs.rdb`.
   Thus:
```
    docker container cp data/word_pairs.rdb mst-counter:/home/opencog/data/mpg_parse.rdb
```

5. Copy the the text corpus into the container. The default config
   files expect it at `text/pair-counted`.  Note that this is the
   place where the input corpus was moved to, during pair-counting,
   so if you just grab that, it will all work.
```
     docker container cp text mst-counter:/home/opencog/
```

6. Start the container: `docker start -i mst-counter`
   This will drop you into a shell prompt inside the container.

7. Revise ownership of the data and text directories. The `docker cp`
   above fails to set these correctly. Inside the container:
```
     sudo chown -R opencog:opencog text
     sudo chown -R opencog:opencog data
```

Manual processing
-----------------
Manual processing consists mostly of setting up an environment, and
then launching the MST counting scripts, and then monitoring progress.
Short instructions:

1. Set up the environment. Just cut-n-pste the below:
```
# Everything we need has already been set up in run-1.
# Leave it as-is, in case you want to experiment there.
# Do the actual MST counting in run-3.
# Avoid confusion by removing the config for earlier and later stages.
cd ~/experiments/
cp -pr run-1 run-3
cd run-3
rm -f 2-pair-conf.sh 4-gram-conf.sh

# Use the default configuration.
source 0-pipeline.sh
source 3-mpg-conf.sh

# Remove scripts for earlier and later stages (avoid confusion)
cd run
rm -rf 2-word-pairs 4-gram-class

# Run the MST counting pipeline in byobu.
cd 3-mst-parsing
```

TBD.

Fully automated
---------------
Most of the above has been condensed into a single script.  This can be
run; it exits when done.


------
