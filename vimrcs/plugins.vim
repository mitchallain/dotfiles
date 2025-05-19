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
  Plug 'mitchallain/neosolarized.nvim', { 'branch': 'main' }  " I removed packer dependency
  " Plug 'ishan9299/nvim-solarized-lua'
  " Plug 'shaunsingh/solarized.nvim'
  Plug 'tjdevries/colorbuddy.nvim'
  " Plug 'altercation/solarized'
  Plug 'norcalli/nvim-colorizer.lua', Cond(has('nvim'))  " colorizer for hex codes, etc

  Plug 'nvim-treesitter/nvim-treesitter', {'do': ':TSUpdate'}
  Plug 'nvim-treesitter/nvim-treesitter-context', Cond(has('nvim'))
  Plug 'nvim-treesitter/nvim-treesitter-textobjects', Cond(has('nvim'))
  Plug 'nvim-treesitter/playground'
  " Plug '~/.vim/plugged/nvim-treesorter'
  Plug 'theprimeagen/harpoon'
  Plug 'mbbill/undotree'

  " A tree explorer plugin for vim.
  Plug 'preservim/nerdtree'
  Plug 'jistr/vim-nerdtree-tabs'  " Makes NERDTree independent of tabs

  " Seamless navigation between tmux panes and vim splits
  Plug 'christoomey/vim-tmux-navigator'
  Plug 'preservim/vimux'

  " Specific language/framework support plug-ins
  Plug 'mitchallain/IEC.vim'
  Plug 'vimjas/vim-python-pep8-indent'
  " Plug 'taketwo/vim-ros'  " gotos
  Plug 'iamcco/markdown-preview.nvim', { 'do': { -> mkdp#util#install() }, 'for': ['markdown', 'vim-plug'], 'tag': 'v0.0.10'}
  Plug 'mzlogin/vim-markdown-toc'
  Plug 'MeanderingProgrammer/render-markdown.nvim'
  " Plug 'OXY2DEV/markview.nvim'
  " Re-execute :call mkdp#util#install() if the above fails to install

  " Git plug-ins
  Plug 'tpope/vim-fugitive'
  " Plug 'airblade/vim-gitgutter'
  Plug 'lewis6991/gitsigns.nvim'
  Plug 'APZelos/blamer.nvim'

  Plug 'tpope/vim-surround'
  Plug 'tpope/vim-dispatch'

  " Commenting plugins
  " Plug 'scrooloose/nerdcommenter'
  " Plug 'tpope/vim-commentary'
  Plug 'numToStr/Comment.nvim'  " current favorite
  Plug 'kkoomen/vim-doge', { 'do': { -> doge#install() } }

  " Show numbers in tab names
  Plug 'mkitt/tabline.vim'

  " LSP PLUG-INS
  " --------------------------------
  " see https://github.com/VonHeikemen/lsp-zero.nvim
  Plug 'neovim/nvim-lspconfig'  " Quickstart configs for built-in LSP client
  Plug 'williamboman/mason.nvim', { 'branch': 'main' }
  Plug 'williamboman/mason-lspconfig.nvim', { 'branch': 'main' } 

  " Autocompletion
  Plug 'hrsh7th/nvim-cmp', { 'branch': 'main' } " auto-completion plugin
  Plug 'hrsh7th/cmp-buffer', { 'branch': 'main' } " nvim-cmp source for buffer words
  Plug 'hrsh7th/cmp-path', { 'branch': 'main' }  " nvim-cmp source for filesystem paths

  " Completion with nvim-cmp
  " https://github.com/hrsh7th/nvim-cmp
  Plug 'onsails/lspkind.nvim'  " prettier completion window
  
  " List of completion sources
  " https://github.com/hrsh7th/nvim-cmp/wiki/List-of-sources
  Plug 'hrsh7th/cmp-nvim-lsp', { 'branch': 'main' }  " LSP source for nvim-cmp
  Plug 'hrsh7th/cmp-nvim-lua', { 'branch': 'main' }  " LSP source for nvim-cmp
  " Plug 'hrsh7th/cmp-cmdline', { 'branch': 'main' }  " nvim-cmp source for vim's cmdline, found this to be intrusive
  Plug 'aspeddro/cmp-pandoc.nvim', { 'branch': 'main' } " nvim-cmp source for markdown / pandoc

  " For luasnip usage
  Plug 'saadparwaiz1/cmp_luasnip' " Snippets source for nvim-cmp
  Plug 'L3MON4D3/LuaSnip', Cond(has('nvim')) " Snippets plugin

  " null-ls - Inject diagnostics and formatting through LSP from supported
  " binaries on PATH. Note this does not provide the binaries, which must be
  " installed seperately with system package manager or lang package manager.
  " Built-in support:
  " https://github.com/jose-elias-alvarez/null-ls.nvim/blob/main/doc/BUILTINS.md
  " main branch no longer supports 0.7 as of 2022-12-07
  Plug 'jose-elias-alvarez/null-ls.nvim', { 'branch': '0.7-compat' }

  " Prettier listings of diagnostics, LSP refs, etc.
  Plug 'kyazdani42/nvim-web-devicons'
  Plug 'folke/trouble.nvim', Cond(has('nvim'), { 'tag': 'v3.6.0' })
  Plug 'folke/todo-comments.nvim', Cond(has('nvim'), { 'branch': 'main' })

  Plug 'mitchallain/toggle-lsp-diagnostics.nvim', { 'branch': 'main' }  " easily toggle virtual text diagnostics on and off
  Plug 'ojroques/nvim-lspfuzzy', { 'branch': 'main' }
  " --------------------------------------
  
  " Debug Adapter Protocol
  Plug 'mfussenegger/nvim-dap'
  Plug 'mfussenegger/nvim-dap-python'
  Plug 'theHamsta/nvim-dap-virtual-text'
  Plug 'rcarriga/nvim-dap-ui'
  Plug 'nvim-neotest/nvim-nio'

  Plug 'nvim-lua/plenary.nvim'  " Lua function library
  Plug 'nvim-telescope/telescope.nvim', { 'tag': '0.1.8' }
  Plug 'nvim-telescope/telescope-fzf-native.nvim', { 'do': 'cmake -S. -Bbuild -DCMAKE_BUILD_TYPE=Release && cmake --build build --config Release && cmake --install build --prefix build', 'branch': 'main', }
  Plug 'kkharji/sqlite.lua'
  Plug 'nvim-telescope/telescope-frecency.nvim'

  Plug 'jbyuki/nabla.nvim'  " floating LaTeX equation rendered

  " Zeavim allows to use the offline documentation browser Zeal from Vim.
  Plug 'KabbAmine/zeavim.vim'

  Plug 'vim-scripts/Tabmerge'

  " Shows marks next to line numbers
  Plug 'kshenoy/vim-signature'

  " ctags and tagbar
  " ensure that 'sudo snap/apt install universal-ctags'
  " Plug 'ludovicchabant/vim-gutentags'
  " Plug 'liuchengxu/vista.vim'
  Plug 'preservim/tagbar'

  " Plug 'github/copilot.vim', { 'branch': 'release' }
  " anduril tings
  Plug 'Exafunction/codeium.vim', { 'tag': '1.12.0' }
  Plug 'olimorris/codecompanion.nvim'
  Plug 'sourcegraph/sg.nvim', { 'do': 'nvim -l build/init.lua' }

  Plug 'jamestthompson3/nvim-remote-containers'
  Plug 'Almo7aya/openingh.nvim'

  Plug 'epwalsh/obsidian.nvim'
  Plug 'peterhoeg/vim-qml'
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
" hi ColorColumn ctermbg=grey guibg=#002b36  " set by neosolarized now

"""""""""""""""""""""""""""""""
" => Colorizer
""""""""""""""""""""""""""""""
let g:colorizer_auto_filetype='vim,lua,css,html'

"""""""""""""""""""""""""""""""
" => fzf - fuzzy finder
""""""""""""""""""""""""""""""
set rtp+=~/.fzf
" Custom fzf command always shows basename, does not care about case,
" and searches basename first, before falling back to complete pathname
nnoremap <c-p> :FZF --keep-right -i --delimiter / --nth -1,..<cr>
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
" => Open in GitHub with openingh.nvim
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
nnoremap <silent> <leader>gh :OpenInGHFileLines<cr>


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

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => devcontainers
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" hi Container guifg=#BADA55 guibg=Black
" if g:currentContainer is set
" TODO: this doesn't work as intended
if exists('g:currentContainer')
    set statusline+=%#Container#%{g:currentContainer}
endif

