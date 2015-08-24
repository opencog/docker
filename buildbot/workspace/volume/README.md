### Keep this directory until a better option is found.

This directory is created to simplify the management of docker volume-container.
By having this volume created as non-root before running docker-compose up, it
is guaranteed that the workspace setup will complete and the artifacts created
by buildbot will be kept in this directory.
