"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Vim Settings
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
set history=500  " number of lines of command history
set nofoldenable

" Enable filetype plugins
filetype plugin on
filetype indent on

" Set to auto read when a file is changed from the outside
set autoread
autocmd FocusGained,BufEnter * checktime

" possibly a little more robust but so far untested
" autocmd FocusGained,BufEnter,CursorHold,CursorHoldI * if mode() != 'c' | checktime | endif

" Neovim defaults to a thin cursor in insert mode
set guicursor=i:block

set rnu
set nu
set mouse=a
set clipboard=unnamed

" Set 7 lines to the cursor - when moving vertically using j/k
set scrolloff=7

" Turn on the Wild menu
set wildmenu

" Ignore compiled files
set wildignore=*.o,*~,*.pyc
if has("win16") || has("win32")
    set wildignore+=.git\*,.hg\*,.svn\*
else
    set wildignore+=*/.git/*,*/.hg/*,*/.svn/*,*/.DS_Store
endif

" Always show current position
set ruler

" Height of the command bar
set cmdheight=1

" A buffer becomes hidden when it is abandoned
set hidden

" Configure backspace so it acts as it should act
set backspace=eol,start,indent
set whichwrap+=<,>,h,l

" Ignore case when searching
set ignorecase

" When searching try to be smart about cases
set smartcase

" Don't highlight search results after <CR>
" with hlsearch, have to use :noh to turn off annoying highlights
set nohlsearch

" Makes search act like search in modern browsers
set incsearch

" Don't redraw while executing macros (good performance config)
set lazyredraw

" For regular expressions turn magic on
set magic

" Show matching brackets when text indicator is over them
set showmatch

" How many tenths of a second to blink when matching brackets
set mat=2

" No annoying sound on errors
set noerrorbells
set novisualbell
set t_vb=
set tm=500

" Properly disable sound on errors on MacVim
if has("gui_macvim")
    autocmd GUIEnter * set vb t_vb=
endif

" Add a bit extra margin to the left
set foldcolumn=1

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Basic Keymaps
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

let mapleader = " "

" Fast saving
nmap <leader>w :w!<cr>

" :W sudo saves the file
" (useful for handling the permission-denied error)
command! W execute 'w !sudo tee % > /dev/null' <bar> edit!

" Ctrl-C behaves like escape, e.g. triggers InsertLeave
imap <C-c> <Esc>

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Colors and Fonts
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Enable syntax highlighting
syntax enable

" Enable 256 colors palette in Gnome Terminal
if $COLORTERM == 'gnome-terminal'
    set t_Co=256
endif

" Set extra options when running in GUI mode
if has("gui_running")
    set guioptions-=T
    set guioptions-=e
    set t_Co=256
    set guitablabel=%M\ %t
endif

" Set utf8 as standard encoding and en_US as the standard language
set encoding=utf8

" Use Unix as the standard file type
set ffs=unix,dos,mac


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Files, backups and undo
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
set nobackup
set nowb
set noswapfile
set undodir=~/.vim/undodir  " this gives undotree access


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Text, tab and indent related
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Use spaces instead of tabs
set tabstop=4  " 1 tab -> 4 spaces
set shiftwidth=0  " this means equal to tabstop
set softtabstop=0  " off
set expandtab
" note the above configuration does not require setting softtabstop

" Be smart when using tabs ;)
set smarttab

" when all else fails, quickly toggle between tabwidths
nnoremap <leader>t2 :setlocal tabstop=2 shiftwidth=0 softtabstop=0 expandtab<cr>
nnoremap <leader>t4 :setlocal tabstop=4 shiftwidth=0 softtabstop=0 expandtab<cr>
nnoremap <leader>t8 :setlocal tabstop=8 shiftwidth=0 softtabstop=0 expandtab<cr>

" only when set wrap
set linebreak
set textwidth=500

set autoindent
set smartindent
set nowrap

set colorcolumn=80,100


""""""""""""""""""""""""""""""
" => Visual mode related
""""""""""""""""""""""""""""""
" Visual mode pressing * or # searches for the current selection
" Super useful! From an idea by Michael Naumann
vnoremap <silent> * :<C-u>call VisualSelection('', '', '')<CR>/<C-R>=@/<CR><CR>
vnoremap <silent> # :<C-u>call VisualSelection('', '', '')<CR>?<C-R>=@/<CR><CR>

" jump into ripgrep search with visual selection
vnoremap <leader>fi :<C-u>call VisualSelection('', '', 'false')<CR>:Rg <C-R>=@/<CR><CR>
nnoremap <leader>fi viw:<C-u>call VisualSelection('', '', 'false')<CR>:Rg <C-R>=@/<CR><CR>


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Keymaps for tabs, windows and buffers
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Toggle highlights when searching on <leader><cr>
map <silent> <leader><cr> :setlocal hlsearch!<cr>

" Fuzzy buffer searching
map <leader>b :Buffers<cr>

" testing tab navigation with leader then num
map <leader>1 1gt
map <leader>2 2gt
map <leader>3 3gt
map <leader>4 4gt
map <leader>5 5gt
map <leader>6 6gt
map <leader>7 7gt
map <leader>8 8gt
map <leader>9 9gt

" Close the current buffer
map <leader>bd :Bclose<cr>:tabclose<cr>gT

" Close all the buffers
map <leader>ba :bufdo bd<cr>

map <leader>l :bnext<cr>
map <leader>h :bprevious<cr>

" Useful mappings for managing tabs
map <leader>tn :tabnew<cr>
map <leader>to :tabonly<cr>
map <leader>tc :tabclose<cr>
map <leader>tm :tabmove

" Let 'tl' toggle between this and the last accessed tab
let g:lasttab = 1
nmap <leader>tl :exe "tabn ".g:lasttab<CR>
au TabLeave * let g:lasttab = tabpagenr()

" Switch CWD to the directory of the open buffer
map <leader>cd :cd %:p:h<cr>:pwd<cr>

" Specify the behavior when switching between buffers
try
  set switchbuf=useopen,usetab,newtab
  set stal=2
catch
endtry

" Return to last edit position when opening files (You want this!)
au BufReadPost * if line("'\"") > 1 && line("'\"") <= line("$") | exe "normal! g'\"" | endif

nnoremap <M-J> :resize +5<cr>
nnoremap <M-K> :resize -5<cr>
nnoremap <M-H> :vertical resize +5<cr>
nnoremap <M-L> :vertical resize -5<cr>

""""""""""""""""""""""""""""""
" => Status line
""""""""""""""""""""""""""""""
" Always show the status line
set laststatus=2

" Format the status line
set statusline=\ %{HasPaste()}%f%m%r%h\ %w\ \ \ \ %l/%L,\ %c


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Editing mappings
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Remap VIM 0 to first non-blank character
" map 0 ^  " this is a bad habit

" Sort selection
vnoremap <leader>so :sort<cr>

" normal mode replaces word under cursor
" visual mode replaces selection
nnoremap <leader>s :%s/\<<C-r><C-w>\>/
vnoremap <leader>s y:%s/<C-r><C-r>"/

" normal mode modify the word under the cursor
" visual mode modify the visual selection
nnoremap <leader>cw :%s/\<<C-r><C-w>\>/<C-r><C-w>
vnoremap <leader>cw y:%s/<C-r><C-r>"/<C-r><C-r>"

" replace word project wide in the quickfix list
nnoremap <leader>qr :cfdo %s/

" open sublime merge
nnoremap <silent> <leader>me :!smerge %s<cr><cr>

" goto 80
nnoremap <leader>\ 80\|

" Move text in visual mode
vnoremap J :m '>+1<cr>gv=gv
vnoremap K :m '<-2<cr>gv=gv

" Join lines without moving cursor
nnoremap J mzJ`z

" Search stays in center
nnoremap n nzzzv
nnoremap N Nzzzv

" Pasting over a word deletes it into the black hole register first
" This assumes that you don't want to keep the replaced text around
vnoremap P "_dP
vnoremap p "_dp

" Yanking into clipboard with leader
nnoremap <leader>y "+y
vnoremap <leader>y "+y
nnoremap <leader>Y "+Y

" yank current buffer path into clipboard
nnoremap <leader>cp :let @+ = expand("%")<cr>

" Don't ever press capital Q, it's the worst place in the universe - the primeagen
nnoremap Q <nop>

" Toggle executability of the file on and off
nnoremap <leader>x :sil !if [ -x % ]; then chmod -x %; else chmod +x %; fi<cr>

" 'Jump' mappings, because the square bracket is hard to reach for me
nnoremap <leader>jf ]m
nnoremap <leader>Jf [m
nnoremap <leader>jr ]M
nnoremap <leader>Jr [M

" Google a selection
vnoremap <C-g> y:sil !google-chrome "? <C-r>""<cr>
nnoremap <C-g> yiw:sil !google-chrome "? <C-r>""<cr>

" Delete trailing white space on save, useful for some filetypes
fun! CleanExtraSpaces()
    let save_cursor = getpos(".")
    let old_query = getreg('/')
    silent! %s/\s\+$//e
    call setpos('.', save_cursor)
    call setreg('/', old_query)
endfun

if has("autocmd")
    autocmd BufWritePre *.txt,*.h,*.cpp,*.cc,*.py,*.sh,*.md :call CleanExtraSpaces()
endif

nnoremap <leader>tw :call CleanExtraSpaces()<cr>


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Spell checking
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Pressing ,ss will toggle and untoggle spell checking
map <leader>ss :setlocal spell!<cr>

" Shortcuts using <leader>
" map <leader>sn ]s
" map <leader>sp [s
" map <leader>sa zg
" map <leader>s? z=


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Misc
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Remove the Windows ^M - when the encodings gets messed up
" noremap <leader>m mmHmt:%s/<C-V><cr>//ge<cr>'tzt'm

" Toggle paste mode on and off
map <leader>pp :setlocal paste!<cr>

" Select recently pasted text
" https://vim.fandom.com/wiki/Selecting_your_pasted_text
nnoremap <expr> gp '`[' . strpart(getregtype(), 0, 1) . '`]'


" https://github.com/tjdevries/config_manager/blob/83b6897e83525efdfdc24001453137c40373aa00/xdg_config/nvim/plugin/keymaps.vim#L45-L53
" Execute a single line or selection of vimscript or lua
function! s:executor() abort
  if &ft == 'lua'
    " echo "test"
    call execute(printf(":lua %s", getline(".")))
  elseif &ft == 'vim'
    exe getline(">")
  endif
endfunction
nnoremap <leader>xl :call <SID>executor()<CR>
vnoremap <leader>xl :<C-w>exe join(getline("'<","'>"),'<Bar>')<CR>

" Execute this file
if !exists('*basic#save_and_exec')
  function! basic#save_and_exec() abort
    if &filetype == 'vim'
      :silent! write
      :source %
    elseif &filetype == 'lua'
      :silent! write
      :luafile %
    endif

    return
  endfunction
endif

nnoremap <leader><leader>xf :call basic#save_and_exec()<CR>


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" => Helper functions
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Returns true if paste mode is enabled
function! HasPaste()
    if &paste
        return 'PASTE MODE  '
    endif
    return ''
endfunction

" Don't close window, when deleting a buffer
command! Bclose call <SID>BufcloseCloseIt()
function! <SID>BufcloseCloseIt()
    let l:currentBufNum = bufnr("%")
    let l:alternateBufNum = bufnr("#")

    if buflisted(l:alternateBufNum)
        buffer #
    else
        bnext
    endif

    if bufnr("%") == l:currentBufNum
        new
    endif

    if buflisted(l:currentBufNum)
        execute("bdelete! ".l:currentBufNum)
    endif
endfunction

function! CmdLine(str)
    call feedkeys(":" . a:str)
endfunction

function! VisualSelection(direction, extra_filter, escape) range
    let l:saved_reg = @"
    execute "normal! vgvy"

    if a:escape == 'false'
        let l:pattern = @"
    else
        let l:pattern = escape(@", "\\/.*'$^~[]")
    endif
    
    let l:pattern = substitute(l:pattern, "\n$", "", "")

    if a:direction == 'gv'
        call CmdLine("Ack '" . l:pattern . "' " )
    elseif a:direction == 'replace'
        call CmdLine("%s" . '/'. l:pattern . '/')
    endif

    let @/ = l:pattern
    let @" = l:saved_reg
endfunction
