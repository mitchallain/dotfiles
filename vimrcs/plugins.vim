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

""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" SECTION 1: VIM-COMPATIBLE PLUGINS (work in both vim & nvim)
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

  " === Fuzzy Finding ===
  Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }
  Plug 'junegunn/fzf.vim'

  " === File Navigation ===
  Plug 'preservim/nerdtree'
  " Makes NERDTree independent of tabs
  Plug 'jistr/vim-nerdtree-tabs'

  " === Git ===
  Plug 'tpope/vim-fugitive'
  " vim-compatible despite .nvim suffix (requires vim 8+)
  Plug 'APZelos/blamer.nvim'

  " === Editing ===
  Plug 'tpope/vim-surround'
  Plug 'tpope/vim-dispatch'
  Plug 'kkoomen/vim-doge', { 'do': { -> doge#install() } }

  " === Tmux Integration ===
  Plug 'christoomey/vim-tmux-navigator'
  Plug 'preservim/vimux'

  " === Language Support ===
  Plug 'mitchallain/IEC.vim'
  Plug 'vimjas/vim-python-pep8-indent'
  " vim-compatible despite .nvim suffix (requires vim 8+)
  Plug 'iamcco/markdown-preview.nvim', { 'do': { -> mkdp#util#install() }, 'for': ['markdown', 'vim-plug'], 'tag': 'v0.0.10'}
  Plug 'mzlogin/vim-markdown-toc'
  " Re-execute :call mkdp#util#install() if the above fails to install

  " === Utilities ===
  Plug 'mbbill/undotree'
  Plug 'mkitt/tabline.vim'
  " Shows marks next to line numbers
  Plug 'kshenoy/vim-signature'
  " ctags sidebar (ensure 'sudo snap/apt install universal-ctags')
  Plug 'preservim/tagbar'
  " Use offline documentation browser Zeal from Vim
  Plug 'KabbAmine/zeavim.vim'
  Plug 'vim-scripts/Tabmerge'
  Plug 'peterhoeg/vim-qml'

  " === Color Schemes (vim-compatible) ===
  " Solarized for vim (modern, actively maintained for vim 8+)
  Plug 'lifepillar/vim-solarized8'

  " === AI Tools (vim-compatible) ===
  Plug 'Exafunction/windsurf.vim'

""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" SECTION 2: NEOVIM-ONLY PLUGINS (require neovim APIs)
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

  " === Color Schemes ===
  " Solarized for neovim (removed packer dependency)
  Plug 'mitchallain/neosolarized.nvim', Cond(has('nvim'), { 'branch': 'main' })
  Plug 'tjdevries/colorbuddy.nvim', Cond(has('nvim'))
  " Colorizer for hex codes, etc
  Plug 'norcalli/nvim-colorizer.lua', Cond(has('nvim'))

  " === Treesitter ===
  Plug 'nvim-treesitter/nvim-treesitter', Cond(has('nvim'), {'do': ':TSUpdate'})
  Plug 'nvim-treesitter/nvim-treesitter-context', Cond(has('nvim'))
  Plug 'nvim-treesitter/nvim-treesitter-textobjects', Cond(has('nvim'))

  " === LSP ===
  " Quickstart configs for built-in LSP client
  Plug 'neovim/nvim-lspconfig', Cond(has('nvim'))
  Plug 'williamboman/mason.nvim', Cond(has('nvim'), { 'branch': 'main' })
  Plug 'williamboman/mason-lspconfig.nvim', Cond(has('nvim'), { 'branch': 'main' })
  " Community fork of null-ls with nvim 0.11+ support
  Plug 'nvimtools/none-ls.nvim', Cond(has('nvim'))
  Plug 'mitchallain/toggle-lsp-diagnostics.nvim', Cond(has('nvim'), { 'branch': 'main' })
  Plug 'ojroques/nvim-lspfuzzy', Cond(has('nvim'), { 'branch': 'main' })

  " === Completion ===
  " Auto-completion plugin
  Plug 'hrsh7th/nvim-cmp', Cond(has('nvim'), { 'branch': 'main' })
  " nvim-cmp source for buffer words
  Plug 'hrsh7th/cmp-buffer', Cond(has('nvim'), { 'branch': 'main' })
  " nvim-cmp source for filesystem paths
  Plug 'hrsh7th/cmp-path', Cond(has('nvim'), { 'branch': 'main' })
  " Prettier completion window
  Plug 'onsails/lspkind.nvim', Cond(has('nvim'))
  " LSP source for nvim-cmp
  Plug 'hrsh7th/cmp-nvim-lsp', Cond(has('nvim'), { 'branch': 'main' })
  " Lua source for nvim-cmp
  Plug 'hrsh7th/cmp-nvim-lua', Cond(has('nvim'), { 'branch': 'main' })
  " nvim-cmp source for markdown/pandoc
  Plug 'aspeddro/cmp-pandoc.nvim', Cond(has('nvim'), { 'branch': 'main' })
  " Snippets source for nvim-cmp
  Plug 'saadparwaiz1/cmp_luasnip', Cond(has('nvim'))
  " Snippets plugin
  Plug 'L3MON4D3/LuaSnip', Cond(has('nvim'))

  " === UI/Diagnostics ===
  Plug 'kyazdani42/nvim-web-devicons', Cond(has('nvim'))
  Plug 'folke/trouble.nvim', Cond(has('nvim'), { 'tag': 'v3.6.0' })
  Plug 'folke/todo-comments.nvim', Cond(has('nvim'), { 'branch': 'main' })

  " === Git ===
  Plug 'lewis6991/gitsigns.nvim', Cond(has('nvim'))
  Plug 'Almo7aya/openingh.nvim', Cond(has('nvim'))

  " === Navigation ===
  " Lua function library
  Plug 'nvim-lua/plenary.nvim', Cond(has('nvim'))
  Plug 'nvim-telescope/telescope.nvim', Cond(has('nvim'), { 'tag': '0.1.8' })
  Plug 'nvim-telescope/telescope-fzf-native.nvim', Cond(has('nvim'), { 'do': 'cmake -S. -Bbuild -DCMAKE_BUILD_TYPE=Release && cmake --build build --config Release && cmake --install build --prefix build', 'branch': 'main' })
  Plug 'kkharji/sqlite.lua', Cond(has('nvim'))
  Plug 'nvim-telescope/telescope-frecency.nvim', Cond(has('nvim'))
  Plug 'stevearc/oil.nvim', Cond(has('nvim'))
  Plug 'theprimeagen/harpoon', Cond(has('nvim'))

  " === Debug Adapter Protocol ===
  Plug 'mfussenegger/nvim-dap', Cond(has('nvim'))
  Plug 'mfussenegger/nvim-dap-python', Cond(has('nvim'))
  Plug 'theHamsta/nvim-dap-virtual-text', Cond(has('nvim'))
  Plug 'rcarriga/nvim-dap-ui', Cond(has('nvim'))
  Plug 'nvim-neotest/nvim-nio', Cond(has('nvim'))

  " === Markdown ===
  Plug 'MeanderingProgrammer/render-markdown.nvim', Cond(has('nvim'))
  " Floating LaTeX equation renderer
  Plug 'jbyuki/nabla.nvim', Cond(has('nvim'))

  " === AI/Workflow ===
  Plug 'olimorris/codecompanion.nvim', Cond(has('nvim'))
  Plug 'sourcegraph/sg.nvim', Cond(has('nvim'), { 'do': 'nvim -l build/init.lua' })
  Plug 'jamestthompson3/nvim-remote-containers', Cond(has('nvim'))
  Plug 'epwalsh/obsidian.nvim', Cond(has('nvim'))

  " === Editing ===
  Plug 'numToStr/Comment.nvim', Cond(has('nvim'))

call plug#end()

"""""""""""""""""""""""""""""""
" => Solarized Color Scheme
""""""""""""""""""""""""""""""
" Use neosolarized for neovim, solarized8 for vim
" Both require truecolor support from the terminal
set termguicolors
set background=dark

" Try to load colorscheme, fail silently if not installed
try
  if has('nvim')
    colorscheme neosolarized
  else
    colorscheme solarized8
  endif
catch /^Vim\%((\a\+)\)\=:E185/
  " Colorscheme not found, using default
endtry

" Make bg transparent
hi Normal ctermbg=NONE guibg=NONE
" hi ColorColumn ctermbg=grey guibg=#002b36  " set by neosolarized now

"""""""""""""""""""""""""""""""
" => fzf - fuzzy finder
""""""""""""""""""""""""""""""
set rtp+=~/.fzf
" Custom fzf command always shows basename, does not care about case,
" and searches basename first, before falling back to complete pathname
" see FZF_DEFAULT_COMMAND for rg default options
nnoremap <c-p> :FZF --keep-right -i --delimiter / --nth -1,..<cr>
" Do not respect ignored files, --no-ignore implies --no-ignore-dot, --no-ignore-exclude, --no-ignore-global, no-ignore-parent and --no-ignore-vcs
nnoremap <c-[> :call fzf#run(fzf#wrap({'source': 'rg -i --files --no-ignore --hidden'}))<cr>
inoremap <expr> <c-x><c-f> fzf#vim#complete#path('rg --files')

" :Rga - Ripgrep search all files, including VCS ignored
command! -bang -nargs=* Rga
  \ call fzf#vim#grep(
  \   'rg --column --line-number --no-heading --color=always --no-ignore-vcs --hidden ' 
  \  . (len(<q-args>) > 0 ? <q-args> : '""'), 1,
  \   <bang>0 ? fzf#vim#with_preview('up:60%')
  \           : fzf#vim#with_preview('right:50%:hidden', '?'),
  \   <bang>0)

" nnoremap <leader>st :BTags<cr>
" nnoremap <leader>sa :Tags<cr>
" nnoremap <leader>sm :Marks<cr>
" nnoremap <leader>sb :Buffers<cr>
" nnoremap <leader>sp :Maps<cr>
" nnoremap <leader>sh :Helptags<cr>


"""""""""""""""""""""""""""""""
" => NERDTree - file browser
""""""""""""""""""""""""""""""
nnoremap <C-t> :NERDTreeTabsToggle<CR>
nnoremap <C-f> :NERDTreeTabsFind<CR> :NERDTreeTabsOpen<CR> 

" disable these default keymaps, since they
" conflict with my vim/tmux navigation scheme
let g:NERDTreeMapJumpNextSibling = ''
let g:NERDTreeMapJumpPrevSibling = ''

let g:NERDTreeRespectWildIgnore = 1
let g:NERDTreeShowHidden = 1
let g:NERDTreeIgnore = ['\.ccls-cache$', '\.pytest_cache$', '__pycache__']


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => tagbar
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
nnoremap <leader>t :TagbarToggle<CR>


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Git gutter (Git diff)
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" let g:gitgutter_enabled=0
" nnoremap <silent> <leader>g :GitGutterToggle<cr>

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => markdownpreview
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" set to 1, nvim will open the preview window after entering the markdown buffer
" default: 0
" let g:mkdp_auto_start = 1
let g:mkdp_auto_close = 0
let g:mkdp_preview_options = {
    \ 'disable_sync_scroll': 1,
    \ }

" set to 1, echo preview page url in command line when open preview page
let g:mkdp_echo_preview_url = 1

" specify browser to open preview page
let g:mkdp_browser = 'google-chrome'

" leader mp open preview page
nnoremap <leader>mp :MarkdownPreview<CR>


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
nnoremap <leader>mm :SignatureToggle<cr>


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => vim-doge
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
let g:doge_doc_standard_python = 'google'
" These will override codeium completion
" let g:doge_mapping_comment_jump_forward = '<Tab>'
" let g:doge_mapping_comment_jump_backward = '<S-Tab>'
let g:doge_comment_jump_modes = ['n', 's']


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => blamer
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
let g:blamer_date_format = '%Y-%m-%d'
nnoremap  <leader>bl :BlamerToggle<CR>


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => copilot
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" alternative keymaps for copilot
" let g:copilot_enabled = 0
" nmap <leader>ce :Copilot enable<CR>
" nmap <leader>cc :Copilot disable<CR>
" imap <silent> <C-s> <Plug>(copilot-suggest)
" imap <silent> <C-d> <Plug>(copilot-dismiss)

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => windsurf/codeium
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Pre-define highlight group to avoid loading order issues
if &t_Co == 256
  hi def CodeiumSuggestion guifg=#808080 ctermfg=244
else
  hi def CodeiumSuggestion guifg=#808080 ctermfg=8
endif

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => undotree
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
let g:undotree_WindowLayout = 2

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => vimux
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
map <leader>vl :VimuxRunLastCommand<CR>
map <Leader>vp :VimuxPromptCommand<CR>
" map <Leader>vt :VimuxRunCommand("pytest " . bufname("%"))<CR>
" map <Leader>vt :VimuxPromptCommand("pytest " . bufname("%"))<CR>
map <Leader>vt :VimuxRunCommand("runPhases build check")<CR>
" map <Leader>vb :VimuxRunCommand("cd " . fnamemodify(bufname("%"), ":p:h") . " && catkin build --this")<CR>
map <Leader>vb :VimuxRunCommand("runPhases build")<CR> 
map <Leader>vc :VimuxRunCommand("runPhases clean configure build")<CR>

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => sourcegraph
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" TODO: replicate <leader>fi with sourcegraph
" vnoremap <leader>fs
" nnoremap <leader>fs <cmd>lua require('sg.extensions.telescope').fuzzy_search_results()<CR>
