" Either 'wrapmargin' is in effect or 'textwidth' but not both.
" Lets try living with 'textwidth' and see what that's like. March 2021
" hmm
set wrapmargin=8
" set textwidth=80
set ts=3
set showmode
set noautoindent
set encoding=utf-8

" Only do this part when compiled with support for autocommands.
if has("autocmd")
  " When editing a file, always jump to the last known cursor position.
  " Don't do it when the position is invalid or when inside an event handler
  " (happens when dropping a file on gvim).
  autocmd BufReadPost *
    \ if line("'\"") > 0 && line("'\"") <= line("$") |
    \  exe "normal g`\"" |
    \ endif

endif " has("autocmd")
