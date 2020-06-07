" BEGIN SETTINGS INSERTED BY AWESOME VIM ON GITHUB
set runtimepath+=~/.vim_runtime

" source ~/.vim_runtime/vimrcs/basic.vim
" source ~/.vim_runtime/vimrcs/filetypes.vim
" source ~/.vim_runtime/vimrcs/plugins_config.vim
" source ~/.vim_runtime/vimrcs/extended.vim
source $VIMRUNTIME/vimrc_example.vim

try
source ~/.vim_runtime/my_configs.vim
catch
endtry
" END SETTINGS INSERTED BY AWESOME VIM ON GITHUB

" AUTO INSTALL PLUG
if empty(glob('~/.vim/autoload/plug.vim'))
  silent !curl -fLo ~/.vim/autoload/plug.vim --create-dirs
    \ https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
  autocmd VimEnter * PlugInstall --sync | source $MYVIMRC
endif

call plug#begin('~/.vim/plugged')
Plug 'junegunn/fzf.vim'
"Plug 'junegunn/limelight.vim'
"Plug 'junegunn/goyo.vim'
call plug#end()

" experiments below, I'll move these as appropriate once I figure out what I like
:set rnu

" FUZZY FILE FINDER
:set rtp+=~/.fzf
nnoremap <c-p> :FZF<cr>
" augroup fzf
"   autocmd!
"   autocmd! FileType fzf
"   autocmd  FileType fzf set laststatus=0 noshowmode noruler
"     \| autocmd BufLeave <buffer> set laststatus=2 showmode ruler
" augroup END

:set tabstop=4

"always match tabsize with < and > commands
:set shiftwidth=0
:set expandtab

" see https://stackoverflow.com/a/1625850/3885499
" moves swap, backup, and undo files from working directory to ~/.vim/tmp/ -
" if it exists
set backupdir=~/.vim/tmp//,.
set directory=~/.vim/tmp//,.
set undodir=~/.vim/tmp//,.

