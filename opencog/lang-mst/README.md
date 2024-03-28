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
   files expect it at `data/mpg-parse.rdb`. The `lang-pairs` docker
   scripts previously dropped off the pairs DB at `data/word-pairs.rdb`.
   Thus:
```
    docker container cp data/word-pairs.rdb mst-counter:/home/opencog/data/mpg-parse.rdb
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

0. If you haven't already done so, then: `docker start -i mst-counter`.

1. Set up the environment. Just cut-n-paste the below:
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

# Prepare to run the MST counting pipeline in byobu.
cd 3-mst-parsing
```

2. Start up a byobu/tmux session, just as before, for the word-pairs:
```
./run-mst-shells.sh
```

3. Use F3 and F4 to bounce between byobu/tmux panels. Navigate to the
   `cogserv` panel. Note that it is loading the word-pair dataset. This
   may take a while -- 5-10 minutes, half-an-hour or longer, depending on
   the dataset.

4. Navigate to the `telnet` panel. Explore. Run `top` in the telnet
   session, to see what the CogServer is doing. It will be idle.

5. Navigate to the `submit` panel, and run `./mst-submit.sh`. This will
   start passing the text files into the system. Their processing will
   stall, however, until the word-pairs are fully loaded.  You can watch
   them in the `telnet` panel.

6. You can toggle to the `cogserv` and print some basic progress stats
   by saying `(monitor-parse-rate "hello world")` and `(cog-report-counts)`

7. Processing will continue for hours or days. Processing is done when
   the all of the telnet connections have closed, and the guile process
   goes idle. Manually exit guile.

8. Run `./compute-mst-marginals.sh` (in the `cogserv` panel, after
   exiting guile.) This computes marginal entropies and MI values,
   needed for the next stage of processing.


Link Parser Demo
----------------
The resulting disjunct database can be used by the Link Grammar parser,
as an "ordinary" dictionary, i.e. one containing per-word expressions.
The only unusual aspect is that word categories are not used; all
connectors are to specific words.

1. The RocksDB database can only be used by one user at a time, and so,
   because the CogServer is already using the database, a copy must be
   made. Go to the `spare` tab, and
```
cd ~/data
cp -pr mpg-parse.rdb mpg-parse-copy.rdb
```

2. Start the Link Grammar parser, using the demo dictionary provided.
   This dictionary is located in the `~/demo-dict-mpg` directory. It
   hard-codes the `mpg-parse-copy.rdb` file location in it.
```
cd ~
link-parser demo-dict-mpg -verbosity=3
```

3. Type in any sentence you wish. If the words appear in the dataset,
   and there are appropriate disjuncts on them, then the sentence will
   parse.

The parser will be slow to startup, as portions of the database are
loaded, including expressions for the `LEFT-WALL`, of which there will
typically be millions. This size of this set is reduced only by
clustering, done in a later stage. Once a lookup is done, the results
are cached, so later access should be faster. The first-time lookup
of words during parsing is likewise slow; subsequent access is cached.

The dictionary is defined by the config files in `~/demo-dict-mpg`.
You can copy and rename this directory as desired; you just have to
start Link Grammar in the same directory where this is placed.
Editing the `storage.dict` file to create a custom config. A typical
change is to alter where the data is coming from. This is the line
that starts with `#define storage-node`.

Using a previously built container
----------------------------------
If you've followed the instructions above, and then stopped docker or
rebooted, and want to restart that container, then here's how:

1. Restart the docker container. Use the `-i` flag to get an
   interactive shell:
```
docker start -i <container-name>
```

2. To run the Link Grammar demo, follow the instructions above.

3. To restart processing after a hiatus, interpolate the earlier
   instructions. Typically, this requires restarting the cogserver,
   and then waiting for data to load. This can take tens of minutes
   or hours.
```
cd ~/experiments/run-3/run/3-mst-parsing
./run-mst-shells.sh
```

Semi-automated
--------------
Some notes about the automation process.

* Processing of texts is held off until all word-pairs are loaded. This
  is done with "gates" -- a mutex, called `mst-gate` is created. It is
  locked during word-pair loading, and unlocked when done. The text
  observers are blocked, waiting for this mutex.

* When the submission of texts for counting has ended, the submit script
  calls `(finish-mst-submit)`.  This can be re-defined to do anything,
  when called. By default its a no-op. The fully-automated scripts define
  it so that the CogServer exits.

* When marginals compution finishes, the `/tmp/mst-marginals-done` file
  is touched. Docker uses this to self-halt the container that's running.
  Yes, this is a non-unique tag, if not running in docker, and it needs
  fixing.


Fully automated
---------------
The above has been condensed into a single script, `./autorun.sh`.
It will start the docker container, move datasets into place, perform
the processing, and halt the container when done.

It expects the following:
* A directory `data/mpg-parse.rdb` containing the pairs dataset. Get
  this by copying `../lang-pairs/data/word-pairs.rdb` from the previous
  pair-counting container.
```
    cp -pr ../lang-pairs/data/word-pairs.rdb data/mpg-parse.rdb
```

* A directory `text/pair-counted` containing the text corpus to process.
  Get this by copying `../lang-pairs/text` into the current directory.
```
    cp -pr ../lang-pairs/text .
```

That's it: the two copies above, then `./autorun.sh`

Upon successful completion, it updates both the `text` and the `data`
directories. The resulting `data/mpg-parse.rdb` will contain the MST
disjuncts and marginals.

It assumes that everything has been correctly configured. If it crashes,
hangs, or if you're just antsy and need to see what's going on inside
the container, just run
```
docker exec -it mst-counter-auto /bin/bash
tmux attach
```
This will attach to the byobu/tmux session; the panel layout is
identical to the manual-mode described above.

------
