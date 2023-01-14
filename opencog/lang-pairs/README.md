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
2. `docker create --name pair-counter -p 17001:17001 -it opencog/lang-pairs`
   Note: the -p flag is external_port:internal_port
3. `docker start -i pair-counter`
4. `cd experiments/run-1`
5. Review the config files; change as desired.
6. `. 0-pipeline.sh`  # i.e. source the contents of this config file.
7. `run/run-tmux.sh`  # Set up multiple byobu terminals.
8. Review the project README's and follow those ...

For example:
A. Go to the `cogsrv` tab, and run `run/2-word-pairs/run-cogserver.sh`
B. Go to the `telnet` tab, and run `rlwrap telnet localhost 17001`
   Then run `help` in the cogserver shell and get oriented.
   Then run `top`  in the cogserver shell, to see the stats.
C. Go to the `submit` tab, and run `run/2-word-pairs/pair-submit.sh`
D. Wait until the telnet stats clear ...
E.
