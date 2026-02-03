Base AtomSpace container
------------------------

The container here includes a copy of the core AtomSpace framework,
including the AtomSpace, the CogServer, the RocksStorageNode and the
CogStorageNode. It does *not* include the nlp or learn subsystems.

# Building an Image
The container can bebuilt by the script in the directory below:
`../docker-build.sh -s`. Alternately, like this:

```
docker build --no-cache -t opencog/atomspace .
```
The build will take 5-20 minutes, depending on your connection.

## Creating a Container
After the image is built, a container needs to be defined.
This can be done as follows:
```
docker create --name my-container \
    -p 17001:17001 -p 18080:18080 -p 18081:18081 \
    -v /PATH/TO/YOUR/WORKING/DIRECTORY/On/Your/PC/:/my-workdir \
    -w /my-workdir \
    -it opencog/atomspace:latest
```

The port number mappings above expose the default `CogServerNode`
port numbers from the container. Port 17001 is the telnet port,
port 18080 is the web port, and port 18081 is an auxiliary port
used by the `atomspace-viz` package to provide visualization services.

The (optional) `-v` flag mounts an external working directory into
the container. The `-w` flag sets it as the default home directory.

Replacing `:latest` with some other tag (e.g. `debian-trixie` or
`ubuntu-20.04`) will create a container for the corresponding image.

## Running a Container
The container can be started with
```
docker start -i my-container
```
or by the provided `./run.sh` script.  This will put you at a bash
prompt in the container.

Since one prompt is not really enough to do any serious work, start
either `byobu` or `tmux`, and open a few terminal shells (F2 on `byobu`,
or `ctrl-b c` on tmux.) The shell script `run-tmux.sh` is provided for
your convenience; it will start five panels.

The provided `start.scm` script will load multiple AtomSpace modules
and start a CogServer at the default ports. Althernately, you can do
this by hand.  In one of the panels, start `guile` and say
```
(use-modules (opencog) (opencog cogserver))
(TriggerLink
	(SetValueLink
		(Concept "foobar") (Predicate "my key") (Number 2 3 4)))
(start-cogserver)
```
Then toggle to another byobu/tmux panel (F3/F4 to move left/right
in `byobu`, or `ctrl-b p`/`ctrl-b n` to move left/right in `tmux`)
```
rlwrap telnet localhost 17001
scm
(TriggerLink
	(ValueOfLink
		(Concept "foobar") (Predicate "my key")))
```
The above should display `(Number 2 3 4)`.
Of course!
