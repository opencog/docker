lang-pairs-auto
---------------
This container fully automates the process of counting word-pairs.
It implements the first stage of the
[language learning](https://github.com/opencog/learn/) pipeline.

A full explanation and demo can be found in the
[lang-pairs](../lang-pairs/README.md) directory.

This container will take half-a-day or several days (or longer) to run,
depending on the quantity of input text.  It will need 4 to 24GB of RAM,
depending on the text.

This container is a pre-requisite building block for subsequent stages
of the learning system.

Steps:

0. Install docker, if you have not already done so:
   `sudo apt install docker.io`
00. Run `../docker-build.sh -a` and `../docker-build.sh -l` to build
   the pre-requisite containers.
1. Copy your text files to the `text-files` directory, right here.
   These text files will then be automatically copied into the Docker
   container, to the directory `text/input-files`. You can skip this
   step; files can also be added after the container has been created;
   see below.
2. The first time, say:
```
        docker build -t opencog/lang-pairs-auto .
```
   To force a rebuild:
```
        docker build --no-cache -t opencog/lang-pairs-auto .
```
3. Next,
   `docker create --name pair-auto -p 17002:17002 -it opencog/lang-pairs-auto`
   Note: the `-p` flag is `external_port:internal_port`. The `-p` flag
   exposes the cogserver.
4. Start the container: `docker start -i pair-auto`
   This will drop you into a shell prompt inside the container.
5. `cd experiments/run-1`
6. Review the config files; change if desired. The defaults are fine
   for an initial run. a Later on, you can copy them to
   `experiments/run-2`, `experiments/run-3` and so on, for modified
   extended runs.
7. `source 0-pipeline.sh`  # Load environment variables from config file.
8. `run/run-tmux.sh`       # Set up multiple byobu terminals.
9. Files can be copied in and out of the container using the
   `docker container cp` command. For example, to place more text
   into the input queue:
```
docker container cp some-book.txt pair-counter:/home/opencog/text/input-pages
```
   See the [docker container docs](https://docs.docker.com/engine/reference/commandline/container/)
   for more info.
10. Review the [language-learning project](https://github.com/opencog/learn)
   README's and follow instructions there ...

OK, so those instructions are overwheling. Here's a simplified digest,
just enough to get the basics for word-pairs running:

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
   browsed with a web browser.
   Go to the `cntl` tab and run `sudo service apache2 start`

H. Go to the `cogsrv` tab, load the browser navigation support:
```
(load "/home/opencog/src/cogprotolab/run-word-pairs/scm/navigate.scm")
(define pair-stars asa)
(define pair-freq (add-pair-freq-api pair-stars))
(define (pair-score EDGE) (pair-freq 'pair-fmi EDGE))
(define pair-nav
	(make-nav pair-stars 'right-duals 'left-duals pair-score 10))
```

I. Verify that the above is not insane. The following should print
   plausible results:
```
(pair-freq 'left-wild-fmi (Word "the"))
(pair-nav 'forward (Word "the"))
(pair-nav 'edge-score (Word "the") (Word "door"))
```

J. On your local machine, connect to the webserver, using the port 8080
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

----
