Base AtomSpace container
------------------------

The container here includes a copy of the core AtomSpace framework,
including the AtomSpace, the CogServer, the RocksStorageNode and the
CogStorageNode. It does *not* include nlp, unify, ure or pln.

## Building

The container is built by the script in the directory below:
`../docker-build.sh -s`.

## Testing
Optional. To verify that the core AtomSpace is working:

* Run the container with `./run.sh`. This will put you into a prompt
  in the container.

* Start `guile` at the bash prompt, and say
```
(use-modules (opencog))
(Concept "foobar" (stv 0.5 0.8))
```

* The CogServer can be started also. To verify this, start either
  byobu or tmux, and open a few terminal shells (F2 on byobu, or
  `ctrl-b c` on tmux.) Then, in one of the panels, start guile
  and say
```
(use-modules (opencog) (opencog cogserver))
(start-cogserver)
(Concept "foobar" (stv 0.5 0.8))
```
Then toggle to another byobu/tmux panel (F3/F4 to move left/right
in byobu, or `ctrl-b p`/`ctrl-b n` to move left/right in tmux)
```
rlwrap telnet localhost 17001
scm
(cog-node 'ConceptNode "foobar")
```
The above should display `(ConceptNode "foobar" (stv 0.5 0.8))`.
Of course!
