-- ============================================================================
-- Nvim-Treesitter Setup (for nvim 0.11+)
-- ============================================================================
-- Note: nvim-treesitter had a major rewrite for nvim 0.11
-- The old 'nvim-treesitter.configs' module no longer exists
-- ============================================================================

-- Setup nvim-treesitter for parser installation
require'nvim-treesitter'.setup {
  -- Directory to install parsers and queries to (prepended to `runtimepath`)
  install_dir = vim.fn.stdpath('data') .. '/site'
}

-- Install parsers for languages we use
-- This replaces the old 'ensure_installed' option
local parsers_to_install = {
  "bash",
  "cmake",
  "cpp",
  "dockerfile",
  "vimdoc",
  "json",
  "latex",
  "lua",
  "markdown",
  "markdown_inline",
  "rust",
  "python",
  "vim",
}

-- Install parsers asynchronously (this is a no-op if already installed)
require('nvim-treesitter').install(parsers_to_install)

-- ============================================================================
-- Treesitter Highlighting (now built into Neovim)
-- ============================================================================
-- NOTE: Treesitter highlighting is DISABLED by default (matching your original config)
-- You can enable it per-buffer with <leader>hh or per-filetype below
--
-- Treesitter highlighting disabled due to issues with C++ and Python
-- Using traditional Vim syntax highlighting instead

-- Optional: Enable treesitter for specific filetypes only
-- Uncomment and customize this list if you want treesitter for certain languages:
-- local treesitter_enabled_fts = { "lua", "rust", "markdown" }
-- vim.api.nvim_create_autocmd('FileType', {
--   pattern = treesitter_enabled_fts,
--   callback = function(args)
--     -- Skip large files (> 100 KB)
--     local max_filesize = 100 * 1024
--     local ok, stats = pcall(vim.loop.fs_stat, vim.api.nvim_buf_get_name(args.buf))
--     if ok and stats and stats.size > max_filesize then
--       return
--     end
--     pcall(vim.treesitter.start, args.buf)
--   end,
-- })

-- Toggle treesitter highlighting for current buffer
vim.keymap.set("n", "<leader>hh", function()
  local buf = vim.api.nvim_get_current_buf()
  -- Check if treesitter is currently active for this buffer
  if vim.treesitter.highlighter.active[buf] then
    vim.treesitter.stop(buf)
    print("Treesitter highlighting disabled")
  else
    vim.treesitter.start(buf)
    print("Treesitter highlighting enabled")
  end
end, { noremap = true, silent = false, desc = "Toggle treesitter highlighting" })

-- ============================================================================
-- Nvim-Treesitter-Textobjects Setup (separate plugin)
-- ============================================================================
require("nvim-treesitter-textobjects").setup {
  select = {
    -- Automatically jump forward to textobj, similar to targets.vim
    lookahead = true,

    -- You can choose the select mode (default is charwise 'v')
    selection_modes = {
      ['@parameter.outer'] = 'v', -- charwise
      ['@function.outer'] = 'V', -- linewise
      ['@class.outer'] = '<c-v>', -- blockwise
    },

    -- Include surrounding whitespace
    include_surrounding_whitespace = true,
  },
  swap = {
    enable = true,
  },
}

-- Textobject keymaps (now set manually)
local ts_select = require("nvim-treesitter-textobjects.select")

-- Select textobjects
vim.keymap.set({ "x", "o" }, "af", function()
  ts_select.select_textobject("@function.outer", "textobjects")
end, { desc = "Select outer function" })

vim.keymap.set({ "x", "o" }, "if", function()
  ts_select.select_textobject("@function.inner", "textobjects")
end, { desc = "Select inner function" })

vim.keymap.set({ "x", "o" }, "ac", function()
  ts_select.select_textobject("@class.outer", "textobjects")
end, { desc = "Select outer class" })

vim.keymap.set({ "x", "o" }, "ic", function()
  ts_select.select_textobject("@class.inner", "textobjects")
end, { desc = "Select inner class" })

vim.keymap.set({ "x", "o" }, "as", function()
  ts_select.select_textobject("@scope", "locals")
end, { desc = "Select language scope" })

-- Swap textobjects
local ts_swap = require("nvim-treesitter-textobjects.swap")

vim.keymap.set("n", "<leader>sal", function()
  ts_swap.swap_next("@parameter.inner")
end, { desc = "Swap parameter with next (left)" })

vim.keymap.set("n", "<leader>saj", function()
  ts_swap.swap_next("@parameter.inner")
end, { desc = "Swap parameter with next (down)" })

vim.keymap.set("n", "<leader>scj", function()
  ts_swap.swap_next("@class.outer")
end, { desc = "Swap class with next" })

vim.keymap.set("n", "<leader>sfj", function()
  ts_swap.swap_next("@function.outer")
end, { desc = "Swap function with next" })

vim.keymap.set("n", "<leader>sah", function()
  ts_swap.swap_previous("@parameter.inner")
end, { desc = "Swap parameter with previous (left)" })

vim.keymap.set("n", "<leader>sak", function()
  ts_swap.swap_previous("@parameter.inner")
end, { desc = "Swap parameter with previous (up)" })

vim.keymap.set("n", "<leader>sck", function()
  ts_swap.swap_previous("@class.outer")
end, { desc = "Swap class with previous" })

vim.keymap.set("n", "<leader>sfk", function()
  ts_swap.swap_previous("@function.outer")
end, { desc = "Swap function with previous" })

-- Sort textobjects
vim.keymap.set("n", "<leader>soc", function()
  local ts_sort = require("nvim-treesitter-textobjects.sort")
  ts_sort.sort("@call.outer")
end, { desc = "Sort function calls" })
