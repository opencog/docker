#! /usr/bin/env guile
-l
!#
;
; start.scm
;
; Loader for fooling with cogserver via Ollama 
; Starts a cogserver that Ollama can interact with.
;
; Usage:
;   guile -l start.scm
;
(use-modules (opencog) (opencog persist))
(use-modules (opencog persist-rocks) (opencog persist-cog))
(use-modules (opencog cogserver))
(use-modules (opencog sensory))

; Pick some web port that won't collide with other systems.
(start-cogserver #:port 17001 #:web 18082)
