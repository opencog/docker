lang-pairs
----------

Steps:
0. Copy your text files to `text-files`. These will then be
   automatically copied into the Docker container, to the
   directory `text/input-files`.
1. First time, say:
```
        docker build -t opencog/lang-pairs .
```
   To force a rebuild:
```
        docker build --no-cache -t opencog/lang-pairs .
```
2. `docker create --name pair-counter -p 8080:80 -p 17002:17002 -it opencog/lang-pairs`
   Note: the -p flag is external_port:internal_port
3. `docker start -i pair-counter`
4. `cd experiments/run-1`
5. Review the config files; change as desired.
6. `. 0-pipeline.sh`  # i.e. source the contents of this config file.
7. `run/run-tmux.sh`  # Set up multiple byobu terminals.
8. Review the project README's and follow those ...

For example:
A. Go to the `cogsrv` tab, and run `run/2-word-pairs/run-cogserver.sh`
B. Go to the `telnet` tab, and run `rlwrap telnet localhost 17002`
   Then run `help` in the cogserver shell and get oriented.
   Then run `top`  in the cogserver shell, to see the stats.
C. Go to the `submit` tab, and run `run/2-word-pairs/pair-submit.sh`
D. Wait until the telnet stats clear ...
   This may take hours or days, depending on the dataset.
E. Go to the `cogsrv` tab, and poke around in the AtomSpace
```
(cog-report-counts)
```
F. Go to the `cogsrv` tab, and perform batch MI calculations
   This may take minutes or hours, depending on the dataset.
```
(define ala (make-any-link-api))
(define aca (add-count-api ala))
(define asa (add-pair-stars aca))
(batch-all-pair-mi asa)
(print-matrix-summary-report asa)
```

G. The word-pair dataset can now be browsed with a simple web browser.
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

I. Verify that the above is not insane. The following should work:
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
