#! /usr/bin/env -S guile -l ./cogserver.scm --
!#
;
; cogserver.scm
;
; Load assorted modules and start the cogserver. Start this by saying
; `./cogserver.scm` at the shell prompt. It will start the cogserver,
; and place you at a guile scheme prompt. The cogserver will stop when
; you exit.
;
; This file is here as an example!  Edit this as you wish, and add
; whatever modules or configuration that you want.
;
; Configurable parameters are pulled from the shell environment.
;
(use-modules (system repl common))
(use-modules (opencog) (opencog logger))
(use-modules (opencog persist))
(use-modules (opencog cogserver))
(use-modules (srfi srfi-1))

; Try fishing a prompt string from the shell environment.
(define env-prompt (getenv "PROMPT"))
(when (not env-prompt)
	(set! env-prompt "scheme@atomspace"))

; Prompt magic, copied from `module/system/repl/common.scm`
; This makes the scheme shell prompt work correctly, even when
; exceptions are caught.
(define (cog-prompt)
	(let ((level (length (cond
				((fluid-ref *repl-stack*) => cdr)
				(else '())))))
		(if (zero? level)
			(string-append env-prompt "> ")
			(format #f "~A [~A]> " env-prompt level))))

(repl-default-prompt-set! cog-prompt)

; Some additional envoronment-variable stuffs.
(define env-port (getenv "PORT"))
(if (not env-port)
	(set! env-port 17001)
	(set! env-port(string->number env-port)))

(define env-web-port (getenv "WEB_PORT"))
(if (not env-web-port)
	(set! env-web-port 18080)
	(set! env-web-port(string->number env-web-port)))

(define env-srv-prompt (getenv "OCPROMPT"))
(when (not env-srv-prompt)
	(set! env-srv-prompt "cogserver> "))

(define env-logfile (getenv "LOGFILE"))
(when (not env-logfile)
	(set! env-logfile "/tmp/cogserver.log"))

; Start the cogserver using the configured parameters.
; Start the cogserver *after* opening the DB and setting frames.
; That way, any remote procs waiting on the socket don't start
; sending data until *after* the DB is opened.
(start-cogserver
	#:port env-port
	#:scmprompt (string-append env-prompt "> ")
	#:prompt env-srv-prompt
	#:logfile env-logfile
	#:web env-web-port
	;;; #:web 0  ;; Uncommenting this will disable the websocket port.
	; Random websites will try to talk to the websockets port, so it
	; should be disabled, if you're not using it. I don't know why it
	; is being targeted.
)

; -----------------------------------------------------------
