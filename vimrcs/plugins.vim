" Adapted from https://github.com/amix/vimrc 
" echo 'plugins.vim'

" See https://github.com/junegunn/vim-plug/wiki/tips#automatic-installation
if empty(glob('~/.vim/autoload/plug.vim'))
  silent !curl -fLo ~/.vim/autoload/plug.vim --create-dirs
    \ https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
  autocmd VimEnter * PlugInstall --sync | source $MYVIMRC
endif

" Function from vim-plug docs "Conditional activation"
" conditional loading of plugins
" https://github.com/junegunn/vim-plug/wiki/tips#conditional-activation
function! Cond(cond, ...)
  let opts = get(a:000, 0, {})
  return a:cond ? opts : extend(opts, { 'on': [], 'for': [] })
endfunction

call plug#begin('~/.vim/plugged')
  " Fuzzy searching
  Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }
  Plug 'junegunn/fzf.vim'

  " Solarized for vim and nvim
  " Plug 'overcache/NeoSolarized'  " requires TrueColor support
  Plug 'MitchAllain/neosolarized.nvim', { 'branch': 'main' }
  Plug 'tjdevries/colorbuddy.nvim'
  Plug 'altercation/solarized'
  Plug 'nvim-treesitter/nvim-treesitter', {'do': ':TSUpdate'}
  Plug 'nvim-treesitter/nvim-treesitter-context', Cond(has('nvim'))
  Plug 'nvim-treesitter/playground'
  " Plug 'itchyny/lightline.vim'

  " A tree explorer plugin for vim.
  Plug 'preservim/nerdtree'
  " Makes NERDTree independent of tabs
  Plug 'jistr/vim-nerdtree-tabs'

  " Seamless navigation between tmux panes and vim splits
  Plug 'christoomey/vim-tmux-navigator'

  " This is a Vim plugin that provides Rust file detection, 
  " syntax highlighting, formatting, Syntastic integration, and more. 
  Plug 'rust-lang/rust.vim'
  Plug 'MitchAllain/IEC.vim'

  " ALE (Asynchronous Lint Engine) is a plugin providing linting
  " (syntax checking and semantic errors)
  " Plug 'dense-analysis/ale'
  
  Plug 'tpope/vim-surround'
  Plug 'tpope/vim-fugitive'
  Plug 'airblade/vim-gitgutter'
  "Plug 'scrooloose/nerdcommenter'
  " Plug 'tpope/vim-commentary'
  Plug 'numToStr/Comment.nvim'
  Plug 'tpope/vim-dispatch'

  " Show numbers in tab names
  Plug 'mkitt/tabline.vim'

  " Nodejs extension host for vim & neovim, load extensions 
  " like VSCode and host language servers
  " Plug 'neoclide/coc.nvim', {'branch': 'release'}
  " Plug 'neoclide/coc-python'
  
  " once you are on the latest stable release (>= v0.7.2)
  " this is the recommended python setup
  Plug 'neovim/nvim-lspconfig'  " Quickstart configs for built-in LSP client
  Plug 'WhoIsSethDaniel/toggle-lsp-diagnostics.nvim', { 'branch': 'main' }  " easily toggle virtual text diagnostics on and off

  " Completion with nvim-cmp
  " https://github.com/hrsh7th/nvim-cmp
  Plug 'hrsh7th/nvim-cmp', { 'branch': 'main' } " auto-completion plugin
  Plug 'onsails/lspkind.nvim'  " prettier completion window
  
  " List of completion sources
  " https://github.com/hrsh7th/nvim-cmp/wiki/List-of-sources
  Plug 'hrsh7th/cmp-nvim-lsp', { 'branch': 'main' }  " LSP source for nvim-cmp
  Plug 'hrsh7th/cmp-buffer', { 'branch': 'main' } " nvim-cmp source for buffer words
  Plug 'hrsh7th/cmp-path', { 'branch': 'main' }  " nvim-cmp source for filesystem paths
  " Plug 'hrsh7th/cmp-cmdline'  " nvim-cmp source for vim's cmdline, found this to be intrusive

  Plug 'aspeddro/cmp-pandoc.nvim', { 'branch': 'main' } " nvim-cmp source for markdown / pandoc
  Plug 'jbyuki/nabla.nvim'  " floating LaTeX equation rendered

  " For luasnip users
  Plug 'saadparwaiz1/cmp_luasnip' " Snippets source for nvim-cmp
  Plug 'L3MON4D3/LuaSnip', Cond(has('nvim')) " Snippets plugin

  Plug 'nvim-lua/plenary.nvim'  " Lua function library
  
  " null-ls - Inject diagnostics and formatting through LSP from supported
  " binaries on PATH. Note this does not provide the binaries, which must be
  " installed seperately with system package manager or lang package manager.
  " Built-in support:
  " https://github.com/jose-elias-alvarez/null-ls.nvim/blob/main/doc/BUILTINS.md
  Plug 'jose-elias-alvarez/null-ls.nvim', { 'branch': 'main' }

  " Prettier listings of diagnostics, LSP refs, etc.
  Plug 'kyazdani42/nvim-web-devicons'
  Plug 'folke/trouble.nvim', Cond(has('nvim'), { 'branch': 'main' })

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
  " Plug 'ap/vim-css-color'
  Plug 'chrisbra/Colorizer'

call plug#end()

"""""""""""""""""""""""""""""""
" => NeoSolarized
""""""""""""""""""""""""""""""
" NeoSolarized works with neovim/vim but requires truecolor
" support from the terminal, usually indicated by $COLORTERM
set termguicolors
colorscheme neosolarized

" Make bg transparent
set background=dark
hi Normal ctermbg=NONE guibg=NONE

"""""""""""""""""""""""""""""""
" => lightline
""""""""""""""""""""""""""""""
" let g:lightline = {
"       \ 'colorscheme': 'solarized',
"       \ 'component_function': {
"       \   'gitbranch': 'FugitiveHead'
"       \ },
"       \ }

"""""""""""""""""""""""""""""""
" => Colorizer
""""""""""""""""""""""""""""""
let g:colorizer_auto_filetype='vim,lua,css,html'

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
" nnoremap <C-f> :NERDTreeTabsFind<CR> :NERDTreeTabsToggle<CR> 

" not a big fan of stepping on this built-in, but I do need this mapping
" nnoremap <C-f> :NERDTreeFind<CR>
nnoremap <C-f> :NERDTreeTabsFind<CR>

" disable these default keymaps, since they
" conflict with my vim/tmux navigation scheme
let g:NERDTreeMapJumpNextSibling = ''
let g:NERDTreeMapJumpPrevSibling = ''

let g:NERDTreeRespectWildIgnore = 1
let g:NERDTreeShowHidden = 1
let g:NERDTreeIgnore = ['\.ccls-cache$', '\.pytest_cache$']


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => tagbar
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
nnoremap <leader>t :TagbarToggle<CR>


""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"" => Ale (syntax checker and linter)
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"let g:ale_linters = {
"\   'python': ['flake8'],
"\}

"let g:ale_fixers = {
"\   'rust': ['rustfmt', 'trim_whitespace', 'remove_trailing_lines'],
"\   'python': ['yapf', 'trim_whitespace']
"\}

"" Optional, configure as-you-type completions
"set completeopt=menu,menuone,preview,noselect,noinsert
"let g:ale_completion_enabled = 1

"nmap <silent> <leader>a <Plug>(ale_next_wrap)

"" Disabling highlighting
"let g:ale_set_highlights = 0

"" Only run linting when saving the file
"let g:ale_lint_on_text_changed = 'never'
"let g:ale_lint_on_enter = 0


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

