
Word-pairs Visualization Server
===============================
The code in this directory configures and runs an AtomSpace
providing access to word-pair correlation data, word-disjunct
data, and word similarity scores.

Instructions
------------
Get your hands on one of these datasets (or create your own; see
the opencog/learn project for instructions how.) Contact Linas
to get these datasets.

* `run-1-marg-tranche-123.rdb` -- Requires 59GB to load word-pairs,
    60 GB to be usable, so not eligible.

* `run-1-t12-tsup-1-1-1.rdb` -- 6.4 GB to load word pairs and
   also disjuncts.  9 minutes to load everything.
   7 GB after computing (w,d) MI

* r13-one-sim200.rdb -- 8.2GB to load word-pairs, word-disjunct
   pairs and similarities.

* r13-all-in-one.rdb -- trimed, modified copy of `r14-sim200.rdb`
     It has the following stuff:
     -- Word pairs, from long ago.  These are trimmed to remove all
        word-pairs with MI of 1.1 or less, leaving 3.9M pairs.
     -- word-pair MI that are appropriate, prior to this trim.
     -- word-disjunct pairs, with shape marginals on them.
     -- word-disjunct MI values. (plain, not including shapes)
     -- similarities for top-ranked 1000 words
     -- Requires 6.5 GB to load word-pairs only.
     -- Grows to 8.3 GB after loading word-disjunct pairs.
     -- Grows to 8.6 GB to load similarities.

Then the usual: git clone and compile and install cogutils, the
atomspace, the cogserver, atomspace-rocks and the learn project.

Next, configure the config files to suit your tastes: edit the
files `0-pipeline.sh`  and `pairs-en-conf.sh` in this directory.
Source them, they set environment variables. To source, say this:
```
. 0-pipeline.sh
. pairs-en-conf.sh
```

Next, start the cogserver. Doing this will automatically load
the datasets:
```
guile -l scm/cogserver-nav.scm
```
When the above gets to the guile prompt, it will be ready to serve
data to the visualizer.

General Exploration
===================
A scheme prompt is directly accessible via the cogserver:
```
rlwrap telnet localhost 19014
scm
(format #t "Hello world\n")
```

There are three types of data in this dataset: word-word pairs,
word-disjunct pairs and word-similarities.  The first example
is for word-word pairs, showing how to find related words.

Many words are NOT in the dataset, so random experimentation
may result in no replies.

To get a list of all word-word pairs, with a given word on the right:
```
(pair-stars 'left-stars (WordNode "end"))
```

This will return a long list, the last part of which is:
```
 (EvaluationLink (ctv 1 0 19)
  (LgLinkNode "ANY")
  (ListLink (WordNode "cold") (WordNode "end")))
 (EvaluationLink (ctv 1 0 10)
  (LgLinkNode "ANY")
  (ListLink (WordNode "seen") (WordNode "end")))
 (EvaluationLink (ctv 1 0 34)
  (LgLinkNode "ANY")
  (ListLink (WordNode "quick") (WordNode "end")))
 (EvaluationLink (ctv 1 0 18)
  (LgLinkNode "ANY")
  (ListLink (WordNode "horrible") (WordNode "end")))
```
Ignore the `ctv` (this is an observation count)

These "pairs" are "edges", the name of the edge is "ANY",
and there's a bunch of numerical data hanging off each edge.

The "relatedness" of a word-pair can be obtained by looking
at it's mutual information (MI). View this as follows:
```
(pair-freq 'pair-fmi
   (Evaluation (LgLink "ANY") (List (Word "horrible") (Word "end"))))
```
The above returns a single floating point number, in the range of
about -20 to about +20, indicating the MI of the pair. The higher,
the better.

Pairs, with the word on the left:
```
(pair-stars 'right-stars (Word "end"))
```

The "other end" of a given word (the other vertex at the
end of an edge) can be gotten with:
```
(pair-stars 'right-duals (Word "end"))
(pair-stars 'left-duals (Word "end"))

Simplified Navigation API
-------------------------
To simplify data access, there's a thin wrapper which can be used to
navigate the word-pair graph. Given a word, it will move "forward",
returning the top N related words, ranked in MI order. The "backward"
move moves left. The edge-score is a single floating point number for
that word-pair.

```
;; Examples
;;
;; Create a navigator for the given matrix and ranking objects
;; In this case, pair-stars and pair-freq ...
;; All of these arguments should be taken from a config file.
;;
(define (pair-score EDGE) (pair-freq 'pair-fmi EDGE))
(define pair-nav
   (make-nav pair-stars 'right-duals 'left-duals pair-score 10))

;; Examples
(pair-nav 'forward (Word "end"))
(pair-nav 'backward (Word "end"))

;; Scores, from the forwards-list
(pair-nav 'edge-score (Word "end") (Word "chapter"))
(pair-nav 'edge-score (Word "end") (Word "notes"))
(pair-nav 'edge-score (Word "end") (Word "badly"))

;; From the backwards list
(pair-nav 'edge-score (Word "rear") (Word "end"))
(pair-nav 'edge-score (Word "far") (Word "end"))
```

Word-disjunct pairs
-------------------
A lot like the above, except:
```
(cset-stars 'right-stars (WordNode "end"))
```

Similarity examples
-------------------
The dataset also contains similarity scores for the top most-frequent
1000 words (the "triangle number" of (1001 x 1000 / 2) pairs; about
half a million).

Raw access:
```
(sim-stars 'left-basis-size)
(sim-stars 'right-duals (Word "end"))
(sim-obj 'describe)
(sim-obj 'pair-similarity (sim-stars 'get-pair (Word "end") (Word "well")))
```
The above is nasty in it's native form, so the wrapper in `navigate.scm`
comes to the rescue:

```
;;  First, build the API:
;;
(define (sim-fmi EDGE)
    (cog-value-ref (sim-obj 'pair-similarity EDGE) 0))
(define (sim-vmi EDGE)
    (cog-value-ref (sim-obj 'pair-similarity EDGE) 1))

(define sim-fmi-nav
   (make-nav sim-stars 'right-duals 'left-duals sim-fmi 10))
(define sim-vmi-nav
   (make-nav sim-stars 'right-duals 'left-duals sim-vmi 10))

;; Now, actually use the API:
;;
(sim-fmi-nav 'forward (Word "end"))
(sim-fmi-nav 'edge-score (Word "end") (Word "out"))
```
The dataset contains two different similarity scores, the "fmi"
fractional-MI and the "vmi" variational-MI. The vmi is just the fmi
offset the the mean of the log of the frequency.

Lots of other similarity scores are possible; they're just not in
this dataset. That is, the dataset just caches some pre-computed
values, as otherwise it is fairly CPU-intensive to compute.
(Typically about 10 sims per second, but you need thousands of
these to see which ones are the best/closest. Almost all words
are NOT similar!)

Documentation
=============
All matrix objects have built-in documentation.  For example:
```
(pair-freq 'help)
(pair-freq 'describe)
(pair-stars 'help)
(pair-stars 'describe)
```

TODO:
-----
This:
```
;;; (cog-atomspace-ro!)  ;; No ... this fails somehow.
```

========
