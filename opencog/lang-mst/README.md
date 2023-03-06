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


Manual processing
-----------------
TBD.

Fully automated
---------------
TBD.

------
