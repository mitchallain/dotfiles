-- openingh.nvim configuration
-- Open file/line in GitHub from neovim

-- Open selected lines in GitHub
vim.keymap.set('n', '<leader>gh', ':OpenInGHFileLines<CR>', { silent = true, desc = 'Open in GitHub' })
