;
; navigate.scm
;
; Quick-n-dirty wrapper for getting word-pair similarity.
; Assumes that there are matrix objets and loaded data
; as descrbied in README.md
;


(use-modules (srfi srfi-1))

; General terminology:
; The "stars" are the edges, the duals are the other vertex.
; So, given a vertex, the left-stars are a list of all edges
; pointing at the vertex.  The right-stars are all the edges
; pointing away from the vertex.  The left-duals are all the
; vertexes that have edges pointing at the target. The right-duals
; are all the vertexes that have edges pointing away from the target.
;
; The MI is the mutual information of an edge. Get it by saying
; `(pair-freq 'pair-fmi edge-atom)`
; -----------------------------------------

; Create a simple object with some methods on it ...
(define (make-nav STARS-OBJ FWD-METHOD BACK-METHOD
                  SCORE-EDGE NUM-TO-SHOW)
"
  make-navigator -- Generic graph navigation object
  See examples for more info.
"
	; Make a short list in sorted order.
	(define (short-list FWD TARGET-VERTEX)

		; Depending on whether FWD is #t or #f,
		; get the edge pointing towards or away from the target.
		(define (get-edge OTHER-VERTEX)
			(if FWD
				(STARS-OBJ 'get-pair TARGET-VERTEX OTHER-VERTEX)
				(STARS-OBJ 'get-pair OTHER-VERTEX TARGET-VERTEX)))

		; Define a compare function that compares two
		; vertexes... assuming both have the target-vertex
		; in common.
		(define (compare-fun VTX-A VTX-B)
			(> (SCORE-EDGE (get-edge VTX-A))
			   (SCORE-EDGE (get-edge VTX-B))))

		; Get the list of all vertexes that are joined
		; by some edge to the target vertex.
		(define tail-verts
			(if FWD
				(STARS-OBJ FWD-METHOD TARGET-VERTEX)
				(STARS-OBJ BACK-METHOD TARGET-VERTEX)))

		; Sort the list above, and then return the
		; first NUM-TO-SHOW of that list.
		(take (sort tail-verts compare-fun) NUM-TO-SHOW))

	; Return an edge-score for the given edge.
	; This is a trivial wrapper.
	(define (escore LEFT-VTX RIGHT-VTX)
		(SCORE-EDGE (STARS-OBJ 'get-pair LEFT-VTX RIGHT-VTX)))

	; Call the various methods on the object.
	(lambda (message . args)
		(case message
			((forward) (short-list #t args))
			((backward) (short-list #f args))
			((edge-score)  (apply escore args))
			(else "Ooops! unknown method!")
		))
)

; -----------------------------------------

; Create a simple object with some methods on it ...
(define (make-navigator STARS-OBJ FWD-METHOD BACK-METHOD
                        RANK-OBJ RANK-METHOD NUM-TO-SHOW)
"
  make-navigator -- Generic graph navigation object
  See examples for more info.
"
	(define (score-edge EDGE) (RANK-OBJ RANK-METHOD EDGE))

	(make-nav STARS-OBJ FWD-METHOD BACK-METHOD score-edge NUM-TO-SHOW)
)

; =========================================================
;; Examples
;;
;; Create a navigator for the given matrix and ranking objects
;; In this case, pair-stars and pair-freq ...
;; All of these arguments should be taken from a config file.
;;
;  (define (pair-score EDGE) (pair-freq 'pair-fmi EDGE))
;  (define pair-nav
;     (make-nav pair-stars 'right-duals 'left-duals pair-score 10))
;
;; Examples
;   (pair-nav 'forward (Word "end"))
;   (pair-nav 'backward (Word "end"))
;
;; Scores, from the forwards-list
;   (pair-nav 'edge-score (Word "end") (Word "chapter"))
;   (pair-nav 'edge-score (Word "end") (Word "notes"))
;   (pair-nav 'edge-score (Word "end") (Word "badly"))
;
;; From the backwards list
;   (pair-nav 'edge-score (Word "rear") (Word "end"))
;   (pair-nav 'edge-score (Word "far") (Word "end"))
;
;; =========================================================
;; Like above, but for word similarity.
;;
;;  First, build the API:
;;
;   (define (sim-fmi EDGE)
;       (cog-value-ref (sim-obj 'pair-similarity EDGE) 0))
;   (define (sim-vmi EDGE)
;       (cog-value-ref (sim-obj 'pair-similarity EDGE) 1))
;
;   (define sim-fmi-nav
;      (make-nav sim-stars 'right-duals 'left-duals sim-fmi 10))
;   (define sim-vmi-nav
;      (make-nav sim-stars 'right-duals 'left-duals sim-vmi 10))
;
;; Now, actually use the API:
;;
;   (sim-fmi-nav 'forward (Word "end"))
;   (sim-fmi-nav 'edge-score (Word "end") (Word "out"))
;
;
;; =========================================================
;; =========================================================
;
;
;;; print documentation:
;   (pair-stars 'describe)
;   (pair-freq 'describe)
;
;;; Examples of vertex weights:
;   (pair-freq 'left-wild-logli (Word "start"))
;   (pair-freq 'right-wild-logli (Word "start"))
;   (pair-freq 'right-wild-fentropy (Word "start"))
;   (pair-freq 'left-wild-fentropy (Word "start"))
;   (pair-freq 'left-wild-fmi (Word "start"))
;   (pair-freq 'right-wild-fmi (Word "start"))
;
;; The above could be placed in a popup menu over a vertex
;; (for example) or elsewhere.
;
;;; Examples of edge weights:
;   (pair-freq 'pair-logli (pair-stars 'get-pair (Word "start") (Word "playing")))
;   (pair-freq 'pair-fmi (pair-stars 'get-pair (Word "start") (Word "playing")))
