lang-pairs
----------
This container provides a basic demo, together with a fully automated
script, for counting of word-pairs in text. It consist of three parts:

* A tutorial demonstrating word-pair counting using the
  [language learning](https://github.com/opencog/learn/) pipeline.
* A script that fully automates the processing of text for pair-counting.
* A simple web-based visualizer for the resulting dataset.

Nearby word-pairs are sampled from input text, and a count is maintained
for how often they are seen. These counts are recorded in the AtomSpace,
and written to a persistent database: the RocksB `StorageNode`. This
database can be reopened at a later time, without having to redo
counting.

Given observed frequencies of word-pairs, the mutual information
between them can be calculated. See Wikipedia,
[pointwise mutual information](https://en.wikipedia.org/wiki/Pointwise_mutual_information).
After counting, batch scripts compute this (see instructions below).
After these are computed, an Apache webserver is started inside this
container. Using a web browser to access it will bring up a simple
word-pair visualizer.

The demo can take half-a-day or several days (or longer) to run,
depending on the quantity of input text.  It will need 4 to 12GB of RAM,
again, depending on the text.

This demos only the very first step of a much longer and more complicated
learning system. That system aims to extract syntactic and semantic
structure from arbitrary data (not just text). The second step can be
found in the [`lang-mst`](../lang-mst) directory.  Docker demos for
later stages will be set up "any day now" (The expression "any day now"
is a sarcastic remark, popular in Texas, meaning "not soon".
The learning system is a work in progress, and is unstable.)

A beginning user should run the demo "manually" the first time, in order
to get familiar with the basic steps. Instructions for a fully automated
script are at the very end of this file.

Basic Setup
-----------
0. Install docker, if you have not already done so:
   `sudo apt install docker.io`
1. Run `../docker-build.sh -a` and `../docker-build.sh -l` to build
   the pre-requisite containers.
2. Create this Docker image:
```
        docker build -t opencog/lang-pairs .
```
3. Create a container (an instance of the image):
   `docker create --name pair-counter -p 8080:80 -p 17002:17002 -it opencog/lang-pairs`
   Note: the `-p` flag is `external_port:internal_port`. The first flag
   exposes the internal webserver on `localhost:8080` and the second
   flag exposes the CogServer to the outside world.

4. Copy your input text files into the container, using the
   `docker container cp` command. The default configuration expects
   these files in the `text/input-pages` directory.  For example:
```
docker container cp some-book.txt pair-counter:/home/opencog/text/input-pages
```
   You can place multiple files into arbitrary subdirectories; all
   files and subdirectories will be explored during counting.
   For more info about docker copy, see the
   [docker container docs](https://docs.docker.com/engine/reference/commandline/container/).

Configuration
-------------
Get familiar with the general outlines of the configuration files
and scripting system.

1. Start the container: `docker start -i pair-counter`
   This will drop you into a shell prompt inside the container.
2. Change ownership of the input text files. Run
   `chown -R opencog:opencog text` ***IMPORTANT! Don't forget to do this!***
3. `cd ~/experiments/run-1`
4. Review the config files; change if desired. The defaults are fine
   for an initial run. Later on, you can copy them to
   `experiments/run-2`, `experiments/run-3` and so on, for modified
   extended runs.
5. `source 0-pipeline.sh`  # Load environment variables from config file.
6. `run/run-tmux.sh`       # Set up multiple byobu terminals.
7. Review the [language-learning project](https://github.com/opencog/learn)
   README's and follow instructions there ...

OK, so those instructions are overwhelming. The next section provides
a simplified digest, just enough to get the basics for word-pairs
running.

Manual Counting
---------------
First-time users should follow these instructions.

A. Go to the `cogsrv` tab, and run `run/2-word-pairs/run-cogserver.sh`

B. Go to the `telnet` tab, and run `rlwrap telnet localhost 17002`
   Then run `help` in the cogserver shell and get oriented.
   Then run `top`  in the cogserver shell, to see the stats.

C. Go to the `submit` tab, and run `run/2-word-pairs/pair-submit.sh`

D. Go to the `cogsrv` tab, and poke around in the AtomSpace, as
   desired. It will take a while for processing to complete. So,
   for example, as you wait, you can look at the Atoms piling up:
```
(cog-report-counts)
```
E. Go to the `telnet` tab, and wait until the telnet stats clear ...
   This may take hours or days, depending on the dataset.
   Basically, the system is busy chewing on the data files, and the
   stats report remains busy until that work is done.
   Each file in `~/text/input-pages` is moved to ~/text/pair-counted`
   after it has been submitted. Thus, you can track progress by comparing
   `find ~/text/input-pages/ |wc` to `find ~/text/pair-counted/ |wc`.

XXX below is auto done already??

F. Go to the `cogsrv` tab, and perform batch MI calculations.
   This may take minutes or hours, depending on the dataset.
   It is best not to start these until counting is done.
```
(define ala (make-any-link-api))
(define aca (add-count-api ala))
(define asa (add-pair-stars aca))
(batch-all-pair-mi asa)
(print-matrix-summary-report asa)
```

G. That's it! The word-pair dataset is now complete.  It can now be
   browsed in two different ways: with a web browser, looking at the
   pairs directly, or with the Link Grammar parser, which will use
   it to create MST parses.

Web Demo
--------
The web demo allows the word-pair dataset to be browsed "directly",
using a web graph visualization package.

1. Go to the `cntl` tab and run `sudo service apache2 start`

2. Go to the `cogsrv` tab, load the browser navigation support:
```
(load "/home/opencog/src/cogprotolab/run-word-pairs/scm/navigate.scm")
(define pair-stars asa)
(define pair-freq (add-pair-freq-api pair-stars))
(define (pair-score EDGE) (pair-freq 'pair-fmi EDGE))
(define pair-nav
	(make-nav pair-stars 'right-duals 'left-duals pair-score 10))
```

3. Verify that the above is not insane. The following should print
   plausible results:
```
(pair-freq 'left-wild-fmi (Word "the"))
(pair-nav 'forward (Word "the"))
(pair-nav 'edge-score (Word "the") (Word "door"))
```

4. On your local machine, connect to the webserver, using the port 8080
   specified on `docker create` line above:
```
http://localhost:8080/
```
   or
```
http://172.17.0.1:8080/
```
   where `172.17.0.1` is the Docker container IP address; will vary,
   in general.

Link Parser Demo
----------------
The word-pair database can be used by the Link Grammar parser to create
Minimum Spanning Tree (MST) parses. These are parses where the links
between words correspond to word-pairs that have appeared in the database.
The parsing is done so as to minimize the total cost; equivalently, to
maximize the total MI of all word-pairs used in the parse. (By design,
the LG parser always minimizes costs; thus, the LG dictionary config file
maps MI values to minus costs.)

1. The RocksDB database can only be used by one user at a time, and so,
   because the CogServer is already using the database, a copy must be
   made. Go to the `spare` tab, and
```
cd ~/data
cp -pr word-pairs.rdb word-pairs-copy.rdb
```

2. Start the Link Grammar parser, using the demo dictionary provided.
   This dictionary hard-codes the `word-pairs-copy.rdb` file location
   in it.
```
link-parser demo-dict-pair
```

3. Type in any sentence you wish. If the words appear in the dataset,
   the sentence will parse. The parse is only as accurate as MST parses
   can be: in general, they are "OK", but not great.


Development Tips
----------------
After pair-counting is completed, the easiest way to restart the
CogServer is to invoke `run-common/cogserver-mst.scm`. This loads
all word-pairs into the AtomSpace, and then provides a guile prompt
to poke around in (It does NOT do any MST processing beyond loading
the pairs.)


Semi-automated Counting
-----------------------
Most of the above has been condensed into a single script.

1. Start the container: `docker start -i pair-counter`
   This will drop you into a shell prompt inside the container.
2. (Optional) Review the config files; change if desired. The defaults
   are fine.
3. Run the `/home/opencog/count-pairs.sh` shell script. This will start
   the same tmux/byobu system as the manual instructions above, except
   that this time, all the various servers will be started
   automatically. Progress can be monitored as described above.
4. The shutdown is currently NOT automated. This allows you to review
   results, before shutting everything down.  If satisfied, close the
   `tmux` session with
```
   tmux kill-session
```
5. The word-pair visualizer setup is *not* automated. If you also want
   that, you'll have the follow the last part of the manual instructions
   above.
6. The next step is MST counting; this is handled in the `lang-mst`
   docker image. You will need to save the database of results, in the
   `data/word-pairs.rdb` directory. Copy it out of the container:
```
docker container cp pair-counter:/home/opencog/data/word_pairs.rdb /your/favorite/place/for/data/
```
   This copy can be done before or after shutting down the container.

7. (Optional) During counting, the input text corpus was copied, file by
   file, to `text/pair-counted`, as each file was submitted for
   counting. These will be needed for the next stage, and you can
   minimize confusion by just saving this, as well.
```
docker container cp pair-counter:/home/opencog/text/pair-counted /your/favorite/place/for/text/
```

Fully-automated Counting
------------------------
This can be fully automated. It assumes everything above is configured
and is working.

1. Place a copy of the input text corpus in the `input-pages` directory.
2. Run `autorun.sh`
3. Wait until done.

To process additional text in an existing container, place the new text
into the `input-pages` directory, and run `autorun.sh -u`. Be sure to
remove the earlier text, as otherwise it will be reprocessed a second time.

----
