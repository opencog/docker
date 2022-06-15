;
; cogserver-nav.scm
;
; Run everything needed to get ready to allow visualization of
; the language-learning word-pairs and word-disjunct datasets.
;
; Starts the CogServer, opens the database, loads the word-pairs,
; disjuncts and similarity scores in the database (which usually
; takes about 10-15 minutes.)
;
(include "cogserver.scm")

; Load up the disjuncts -- this can take over half an hour!
(display "Fetch all data. This may take 5-15 minutes.\n")

; Load the word-pairs.
(define pair-obj (make-any-link-api))
(pair-obj 'fetch-pairs)
(define pair-stars (add-pair-stars pair-obj))
(print-matrix-summary-report pair-stars)
(define pair-freq (add-pair-freq-api pair-stars))

; Load the word-disjunct data.
(define cset-obj (make-pseudo-cset-api))
(cset-obj 'fetch-pairs)
(define cset-stars (add-pair-stars cset-obj))
(print-matrix-summary-report cset-stars)
(define cset-freq (add-pair-freq-api cset-stars))

; Load the similarity scores
(define (do-add-similarity-api LLOBJ)
   (define SIM-ID "shape-mi")
   (add-similarity-api LLOBJ #f SIM-ID))

(define sim-obj (do-add-similarity-api cset-obj))
(sim-obj 'fetch-pairs)
(define sim-stars (add-pair-stars sim-obj))

; Close the database to avoid accidental damage.
(cog-close storage-node)

; Sigh It would be nice to mark the atomspace read-only,
; but that breaks things. Oh well.
;;; (cog-atomspace-ro!)
;;; (cog-push-atomspace)

; Now load additional navigation tools.
(include "navigate.scm")
