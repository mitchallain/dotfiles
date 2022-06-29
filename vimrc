" AUTO INSTALL PLUG
" This needs to come before sourcing configuration scripts
" for runtime and package path setup.
if empty(glob('~/.vim/autoload/plug.vim'))
  silent !curl -fLo ~/.vim/autoload/plug.vim --create-dirs
    \ https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
  autocmd VimEnter * PlugInstall --sync | source $MYVIMRC
endif

call plug#begin('~/.vim/plugged')
  " Fuzzy searching
  Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }
  Plug 'junegunn/fzf.vim'

  " Solarized for vim and nvim
  Plug 'overcache/NeoSolarized'
  Plug 'shaunsingh/solarized.nvim'

  " A tree explorer plugin for vim.
  Plug 'preservim/nerdtree'

  " Seamless navigation between tmux panes and vim splits
  Plug 'christoomey/vim-tmux-navigator'

  " This is a Vim plugin that provides Rust file detection, 
  " syntax highlighting, formatting, Syntastic integration, and more. 
  Plug 'rust-lang/rust.vim'

  " ALE (Asynchronous Lint Engine) is a plugin providing linting
  " (syntax checking and semantic errors)
  Plug 'dense-analysis/ale'
  
  Plug 'tpope/vim-surround'
  Plug 'tpope/vim-fugitive'
  Plug 'airblade/vim-gitgutter'
  "Plug 'scrooloose/nerdcommenter'
  Plug 'tpope/vim-commentary'

  " Nodejs extension host for vim & neovim, load extensions 
  " like VSCode and host language servers
  Plug 'neoclide/coc.nvim', {'branch': 'release'}
  Plug 'KabbAmine/zeavim.vim'

  "PYTHON
  Plug 'vimjas/vim-python-pep8-indent'
  Plug 'neoclide/coc-python'

call plug#end()

" BEGIN SETTINGS INSERTED BY AWESOME VIM ON GITHUB
" must follow plug#end for package path setup (NeoVim solarized)
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
nnoremap <Leader>e :e $MYVIMRC<CR>
" " Reload vimr configuration file
nnoremap <Leader>r :source $MYVIMRC<CR>

nnoremap <Leader>s :%s/\<<C-r><C-w>\>/

" END PERSONAL CUSTOMIZATIONS


" FUZZY FILE FINDER
:set rtp+=~/.fzf
nnoremap <c-p> :FZF<cr>
" augroup fzf
"   autocmd! FileType fzf
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

