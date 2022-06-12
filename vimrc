" BEGIN SETTINGS INSERTED BY AWESOME VIM ON GITHUB
"
set runtimepath+=~/.vim_runtime

source ~/.vim_runtime/vimrcs/basic.vim
source ~/.vim_runtime/vimrcs/filetypes.vim
source ~/.vim_runtime/vimrcs/plugins_config.vim
" source ~/.vim_runtime/vimrcs/extended.vim

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
  Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }
  Plug 'junegunn/fzf.vim'
  "Plug 'junegunn/limelight.vim'
  "Plug 'junegunn/goyo.vim'
  Plug 'altercation/vim-colors-solarized'
  Plug 'preservim/nerdtree'
  Plug 'christoomey/vim-tmux-navigator'
  Plug 'rust-lang/rust.vim'
  Plug 'dense-analysis/ale'
  Plug 'tpope/vim-surround'
  Plug 'airblade/vim-gitgutter'
  "Plug 'scrooloose/nerdcommenter'
  Plug 'tpope/vim-commentary'
  Plug 'neoclide/coc.nvim', {'branch': 'release'}
  Plug 'KabbAmine/zeavim.vim'

  "PYTHON
  Plug 'vimjas/vim-python-pep8-indent'
  Plug 'neoclide/coc-python'

call plug#end()

" BEGIN PERSONAL CUSTOMIZATIONS AND EXPERIMENTS
:set rnu
:set nu
:set mouse=a

" see vim-runtime/vimrcs/filetypes.vim for ft specific tab size
:set tabstop=4

"always match tabsize with < and > commands
:set shiftwidth=0
:set expandtab

:set clipboard=unnamed

" Edit vimr configuration file
nnoremap <Leader>ve :e $MYVIMRC<CR>
" " Reload vimr configuration file
nnoremap <Leader>vr :source $MYVIMRC<CR>

set background=dark
colorscheme solarized

" END PERSONAL CUSTOMIZATIONS



" FUZZY FILE FINDER
:set rtp+=~/.fzf
nnoremap <c-p> :FZF<cr>
" augroup fzf


"   autocmd  FileType fzf set laststatus=0 noshowmode noruler
"     \| autocmd BufLeave <buffer> set laststatus=2 showmode ruler
" augroup END


" HIDE VIM SWAP, BKUP, UNDO FILES
" attempt to create vim backup at startup
" https://stackoverflow.com/a/1549318
silent !mkdir ~/.vim/tmp > /dev/null 2>&1
" see https://stackoverflow.com/a/1625850/3885499
" moves swap, backup, and undo files from working directory to ~/.vim/tmp/ -
" if it exists
set backupdir=~/.vim/tmp//,.
set directory=~/.vim/tmp//,.
set undodir=~/.vim/tmp//,.

