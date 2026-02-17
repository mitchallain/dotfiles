-- nvim-remote-containers configuration
-- Shows current container in statusline

vim.cmd([[
" hi Container guifg=#BADA55 guibg=Black
" if g:currentContainer is set
" TODO: this doesn't work as intended
if exists('g:currentContainer')
    set statusline+=%#Container#%{g:currentContainer}
endif
]])
