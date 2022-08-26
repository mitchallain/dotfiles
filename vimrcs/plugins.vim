" Adapted from https://github.com/amix/vimrc 
" echo 'plugins.vim'

" See https://github.com/junegunn/vim-plug/wiki/tips#automatic-installation
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
  " Plug 'overcache/NeoSolarized'  " requires TrueColor support
  Plug 'MitchAllain/neosolarized.nvim'
  Plug 'tjdevries/colorbuddy.nvim'
  Plug 'altercation/solarized'
  Plug 'nvim-treesitter/nvim-treesitter', {'do': ':TSUpdate'}

  " A tree explorer plugin for vim.
  Plug 'preservim/nerdtree'
  " Makes NERDTree independent of tabs
  Plug 'jistr/vim-nerdtree-tabs'

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
  Plug 'tpope/vim-dispatch'

  " Show numbers in tab names
  Plug 'mkitt/tabline.vim'

  " Nodejs extension host for vim & neovim, load extensions 
  " like VSCode and host language servers
  " Plug 'neoclide/coc.nvim', {'branch': 'release'}
  " Plug 'neoclide/coc-python'
  
  " once you are on the latest stable release (>= v0.7.2)
  " this is the recommended python setup
  Plug 'neovim/nvim-lspconfig'
  Plug 'WhoIsSethDaniel/toggle-lsp-diagnostics.nvim'
  Plug 'hrsh7th/nvim-cmp'  " auto-completion plugin
  Plug 'hrsh7th/cmp-nvim-lsp'  " LSP source for nvim-cmp
  Plug 'saadparwaiz1/cmp_luasnip' " Snippets source for nvim-cmp
  Plug 'L3MON4D3/LuaSnip' " Snippets plugin

  Plug 'nvim-lua/plenary.nvim'
  Plug 'jose-elias-alvarez/null-ls.nvim'

  " Zeavim allows to use the offline documentation browser Zeal from Vim.
  Plug 'KabbAmine/zeavim.vim'

  Plug 'kkoomen/vim-doge', { 'do': { -> doge#install() } }
  Plug 'vim-scripts/Tabmerge'

  " Markdown Authoring
  Plug 'iamcco/markdown-preview.nvim', { 'do': { -> mkdp#util#install() }, 'for': ['markdown', 'vim-plug']}

  "PYTHON
  Plug 'vimjas/vim-python-pep8-indent'

  " Shows marks next to line numbers
  Plug 'kshenoy/vim-signature'

  Plug 'taketwo/vim-ros'

  " ctags and tagbar
  " ensure that 'sudo snap/apt install universal-ctags'
  " Plug 'ludovicchabant/vim-gutentags'
  " Plug 'liuchengxu/vista.vim'
  Plug 'preservim/tagbar'

  " Plug 'github/copilot.vim'
  Plug 'ap/vim-css-color'

call plug#end()

"""""""""""""""""""""""""""""""
" => NeoSolarized
""""""""""""""""""""""""""""""
" NeoSolarized works with neovim/vim but requires truecolor
" support from the terminal, usually indicated by $COLORTERM
if $COLORTERM == 'truecolor'
    set termguicolors
    colorscheme neosolarized
else
    " colorscheme solarized
    " This seems to work for some odd reason on mac terminal.app
    colorscheme neosolarized
endif

" Make bg transparent
set background=dark
hi Normal ctermbg=NONE guibg=NONE

"""""""""""""""""""""""""""""""
" => fzf - fuzzy finder
""""""""""""""""""""""""""""""
set rtp+=~/.fzf
nnoremap <c-p> :FZF<cr>

"""""""""""""""""""""""""""""""
" => NERDTree - file browser
""""""""""""""""""""""""""""""
" nnoremap <leader>n :NERDTreeFocus<CR>
" nnoremap <C-n> :NERDTree<CR>

" nnoremap <C-t> :NERDTreeToggle<CR>
nnoremap <C-t> :NERDTreeTabsToggle<CR>

" not a big fan of stepping on this built-in, but I do need this mapping
" nnoremap <C-f> :NERDTreeFind<CR>
nnoremap <C-f> :NERDTreeTabsFind<CR>

" disable these default keymaps, since they
" conflict with my vim/tmux navigation scheme
let g:NERDTreeMapJumpNextSibling = ''
let g:NERDTreeMapJumpPrevSibling = ''


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => tagbar
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
nnoremap <leader>t :TagbarToggle<CR>


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Ale (syntax checker and linter)
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
let g:ale_linters = {
\   'python': ['flake8'],
\}

let g:ale_fixers = {
\   'rust': ['rustfmt', 'trim_whitespace', 'remove_trailing_lines'],
\   'python': ['yapf', 'trim_whitespace']
\}

" Optional, configure as-you-type completions
set completeopt=menu,menuone,preview,noselect,noinsert
let g:ale_completion_enabled = 1

nmap <silent> <leader>a <Plug>(ale_next_wrap)

" Disabling highlighting
let g:ale_set_highlights = 0

" Only run linting when saving the file
let g:ale_lint_on_text_changed = 'never'
let g:ale_lint_on_enter = 0


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Git gutter (Git diff)
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
let g:gitgutter_enabled=0
nnoremap <silent> <leader>g :GitGutterToggle<cr>


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => markdownpreview
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" set to 1, nvim will open the preview window after entering the markdown buffer
" default: 0
" let g:mkdp_auto_start = 1

" set to 1, echo preview page url in command line when open preview page
let g:mkdp_echo_preview_url = 1

" specify browser to open preview page
let g:mkdp_browser = 'google-chrome'


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => vim surround
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
autocmd FileType markdown let b:surround_105 = "*\r*" " i = 105
autocmd FileType markdown let b:surround_98 = "**\r**" " b = 98
autocmd FileType markdown let b:surround_99 = "```\n\r\n```" " b = 98

" not sure if this is working yet
autocmd FileType python let b:surround_100 = "\"\"\"\r\"\"\"" " d = 100
autocmd FileType python let b:surround_115 = "'''\r'''" " s = 115


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => vim-showmarks
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
nnoremap <leader>m :SignatureToggle<cr>


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => vim-doge
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
let g:doge_doc_standard_python = 'google'


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => lsp 
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" autocmd BufEnter * lua vim.diagnostic.disable()

