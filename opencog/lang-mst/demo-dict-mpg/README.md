Demo Atomese MST dictionary
---------------------------
This is an example configuration for an AtomSpace-backed dictionary,
configured for parsing using disjuncts that were mined from MST parses.
Each disjunct is assigned a cost, based on it's MI.

For general info about Link Grammar dictionaries running out of the
AtomSpace, consult the Link Grammar documentation.

* The `4.0.affix` file specifies punctuation that will be automatically
  split off of words.

* The `storage.dict` file specifies assorted configuration data,
  including the location of MI data, and how to scale it to obtain
  LG-style "costs".

Both of the above files are text files.  To adjust the location of the
dictionary, edit the `storage.dict` file and change the line that starts
with `#define storage-node`.
