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
  " NeoSolarized requires TrueColor support
  Plug 'overcache/NeoSolarized'
  Plug 'altercation/solarized'
  " Plug 'shaunsingh/solarized.nvim'
  " Plug 'altercation/vim-colors-solarized'

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
  
  " Zeavim allows to use the offline documentation browser Zeal from Vim.
  Plug 'KabbAmine/zeavim.vim'

  Plug 'kkoomen/vim-doge', { 'do': { -> doge#install() } }
  Plug 'vim-scripts/Tabmerge'
  Plug 'iamcco/markdown-preview.nvim', { 'do': { -> mkdp#util#install() }, 'for': ['markdown', 'vim-plug']}

  "PYTHON
  Plug 'vimjas/vim-python-pep8-indent'
  Plug 'neoclide/coc-python'

  " Shows marks next to line numbers
  Plug 'kshenoy/vim-signature'

call plug#end()

" BEGIN SETTINGS INSERTED BY AWESOME VIM ON GITHUB
" must follow plug#end for package path setup (NeoVim solarized)
" NOTE: I am using this repo for a quick setup and template for config,
"       but I will create a more customized setup once I am more used to
"       vim/nvim. For now, focus is on modifying the options to suit me, and
"       commenting out options which are not useful.
set runtimepath+=~/.vim_runtime
source ~/.vim_runtime/vimrcs/basic.vim
source ~/.vim_runtime/vimrcs/filetypes.vim
source ~/.vim_runtime/vimrcs/plugins_config.vim
" source ~/.vim_runtime/vimrcs/extended.vim
" END SETTINGS INSERTED BY AWESOME VIM ON GITHUB


" SETTINGS INBOX - TODO SORT INTO VIMRCS
" Edit vimr configuration file
" nnoremap <leader>e :e $MYVIMRC<CR>
nnoremap <leader>e :e ~/.vimrc<CR>
" " Reload vimr configuration file
nnoremap <leader>r :source $MYVIMRC<CR>

" normal mode replaces word under cursor
" visual mode replaces selection
nnoremap <leader>s :%s/\<<C-r><C-w>\>/
vnoremap <leader>s y:%s/<C-r><C-r>"/

" I do sort of prefer the thin cursor in neovim,
" but not even to justify the inconsistency with vim
set guicursor=i:block

" Make bg transparent
" hi Normal ctermbg=NONE guibg=NONE
" END SETTINGS INBOX


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

