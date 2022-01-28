set wrapmargin=8
set ts=3
set showmode
set noautoindent
set encoding=utf-8

" Sets the color scheme to something easier on my eyes.
colo evening

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
                                                                                
" /etc/vimrc ends here
