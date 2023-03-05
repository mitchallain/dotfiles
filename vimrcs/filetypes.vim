" Adapted from https://github.com/amix/vimrc 

" echo 'filetypes.vim'

""""""""""""""""""""""""""""""
" => Source Code Files
""""""""""""""""""""""""""""""
" Strip whitespace
au BufRead,BufNewFile *.py,*.pyw,*.c,*.h,*.cpp match BadWhitespace /\s\+$/

""""""""""""""""""""""""""""""
" => Python section
""""""""""""""""""""""""""""""
" let python_highlight_all = 1
" au FileType python syn keyword pythonDecorator True None False self
au FileType python setlocal shiftwidth=0 tabstop=4 softtabstop=0 expandtab

au BufNewFile,BufRead *.jinja set syntax=htmljinja
au BufNewFile,BufRead *.mako set ft=mako


""""""""""""""""""""""""""""""
" => C++ section
""""""""""""""""""""""""""""""
au FileType cpp setlocal shiftwidth=2 tabstop=2 softtabstop=0 expandtab
au FileType cpp let b:dispatch = 'make -C build'
" au FileType cpp nnoremap <leader>c :Dispatch! make -C build<cr>

" TODO: remove in favor of null-ls integration
" apply clang-format to selection with Ctrl K
" see https://clang.llvm.org/docs/ClangFormat.html#vim-integration
" au Filetype cpp map <leader>fo :pyf ~/bin/clang-format.py<cr>

" automatic clang-format on save
function! Formatonsave()
  let l:formatdiff = 1
  " this is a symlink
  pyf ~/bin/clang-format.py
endfunction
" autocmd BufWritePre *.h,*.cc,*.cpp call Formatonsave()

""""""""""""""""""""""""""""""
" => CMake section
""""""""""""""""""""""""""""""
" this doesn't really work when build already exists
au filetype cmake let b:dispatch = 'mkdir build; cd build; cmake ..'

""""""""""""""""""""""""""""""
" => Rust section
""""""""""""""""""""""""""""""
autocmd BufNewFile,BufRead *.rs set filetype=rust
au FileType rust setlocal shiftwidth=0 tabstop=4 softtabstop=0 expandtab

""""""""""""""""""""""""""""""
" => XML section
""""""""""""""""""""""""""""""
autocmd BufNewFile,BufRead *.launch set filetype=xml
au FileType xml setlocal shiftwidth=0 tabstop=2 softtabstop=0 expandtab

""""""""""""""""""""""""""""""
" => IEC 61131-3 ST section
""""""""""""""""""""""""""""""
au BufNewFile,BufRead *.st,*.ST set filetype=iec
au FileType iec setlocal shiftwidth=0 tabstop=4 softtabstop=0

""""""""""""""""""""""""""""""
" => YAML section
""""""""""""""""""""""""""""""
autocmd BufNewFile,BufRead *.yaml,*.yml,*.rosinstall set filetype=yaml
au FileType yaml setlocal shiftwidth=0 tabstop=2 softtabstop=0 expandtab


""""""""""""""""""""""""""""""
" => JavaScript section
"""""""""""""""""""""""""""""""
au FileType javascript call JavaScriptFold()
au FileType javascript setl nocindent

au FileType javascript,typescript imap <C-t> console.log();<esc>hi
au FileType javascript,typescript imap <C-a> alert();<esc>hi

au FileType javascript,typescript inoremap <buffer> $r return 
au FileType javascript,typescript inoremap <buffer> $f // --- PH<esc>FP2xi

function! JavaScriptFold() 
    setl foldmethod=syntax
    setl foldlevelstart=1
    syn region foldBraces start=/{/ end=/}/ transparent fold keepend extend

    function! FoldText()
        return substitute(getline(v:foldstart), '{.*', '{...}', '')
    endfunction
    setl foldtext=FoldText()
endfunction


""""""""""""""""""""""""""""""
" => CoffeeScript section
"""""""""""""""""""""""""""""""
function! CoffeeScriptFold()
    setl foldmethod=indent
    setl foldlevelstart=1
endfunction
au FileType coffee call CoffeeScriptFold()

au FileType gitcommit call setpos('.', [0, 1, 1, 0])


""""""""""""""""""""""""""""""
" => Shell section
""""""""""""""""""""""""""""""
" Google uses 2 spaces
" However, I prefer 4, which matches the default shell config files in Ubuntu
" e.g., .bashrc
au BufNewFile,BufRead *.def set filetype=sh
au FileType sh setlocal tabstop=4 shiftwidth=0 softtabstop=0 expandtab


""""""""""""""""""""""""""""""
" => Twig section
""""""""""""""""""""""""""""""
autocmd BufRead *.twig set syntax=html filetype=html


""""""""""""""""""""""""""""""
" => Markdown
""""""""""""""""""""""""""""""
let vim_markdown_folding_disabled = 1
au Filetype markdown setlocal wrap


""""""""""""""""""""""""""""""
" => Vim
""""""""""""""""""""""""""""""
" automatically source vimrc when changes are made
" see also myconfig for remaps to edit and source vimrc
augroup myvimrc
    au!
    au BufWritePost *.vim,.vimrc,_vimrc,vimrc,.gvimrc,_gvimrc,gvimrc so $MYVIMRC | if has('gui_running') | so $MYGVIMRC | endif
augroup END

""""""""""""""""""""""""""""""
" => Groovy
""""""""""""""""""""""""""""""
au BufNewFile,BufRead Jenkinsfile set filetype=groovy
