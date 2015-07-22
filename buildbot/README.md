# Continuese Integration

These directory carries Continuese Integration and documentation-generation
related configurations. Buildbot is used for both.

## Directory Structure
        ├─master     : has configuration for buildbot master.
        ├─slaves     : has directories for building images for
          ├─latest   : has configuring for all in use buildbot slaves
          ├─precise  : not in use & unmaintained for now.
          ├─utopic   : not in use & unmaintained for now.


The buildbot that is running this configuration is found [here](http://buildbot.opencog.org:8010/)

## Usage
Assuming you have installed docker and docker-compose run
```
docker-compose up
```
In your browser go to http://localhost:8010/waterfall


### TODO:
1. Add relex unit testing
2. Add integration testing between opencog and relex
3. Add integration testing between opencog and unity3d-opencog-game
4. Add support for offline run of unit tests on developer machine using their
   local clone.
5. Auto update buildslaves installation of cogutils and/or atomspace when build
   passes for these repositories.
