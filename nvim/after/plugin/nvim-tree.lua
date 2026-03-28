-- nvim-tree.lua configuration
-- Replaces NERDTree for neovim

-- Disable netrw at the very start (recommended by nvim-tree)
vim.g.loaded_netrw = 1
vim.g.loaded_netrwPlugin = 1

-- Custom keymaps function
local function on_attach(bufnr)
  local api = require('nvim-tree.api')

  -- Default mappings
  api.config.mappings.default_on_attach(bufnr)

  local function opts(desc)
    return { desc = 'nvim-tree: ' .. desc, buffer = bufnr, noremap = true, silent = true, nowait = true }
  end

  -- Custom keymap: C to change directory (cd)
  vim.keymap.set('n', 'C', api.tree.change_root_to_node, opts('CD'))

  -- Remove default 'p' keymap and add custom one to move cursor to parent node
  vim.keymap.del('n', 'p', { buffer = bufnr })
  vim.keymap.set('n', 'p', api.node.navigate.parent, opts('Parent Node'))

  -- Remap refresh from R to r
  vim.keymap.del('n', 'R', { buffer = bufnr })
  vim.keymap.set('n', 'r', api.tree.reload, opts('Refresh'))

  -- Override default <C-t> (open in new tab) to toggle the tree instead
  vim.keymap.set('n', '<C-t>', api.tree.toggle, opts('Toggle'))
end

-- Setup nvim-tree
require("nvim-tree").setup({
  on_attach = on_attach,
  -- Sorting options
  sort = {
    sorter = "case_sensitive",
  },

  -- View configuration
  view = {
    width = 30,
    side = "left",
  },

  -- Renderer options
  renderer = {
    group_empty = true,
    highlight_git = false,  -- Disable git-based coloring, use file type only
    special_files = {},  -- Disable special highlighting for files like README.md
    symlink_destination = true,  -- Show where symlinks point to
    icons = {
      show = {
        git = true,
        folder = false,
        file = false,
        folder_arrow = true,
      },
      git_placement = "after",  -- Show git status after filename
      glyphs = {
        git = {
          unstaged = "(M)",
          staged = "(S)",
          unmerged = "(U)",
          renamed = "(R)",
          untracked = "(?)",
          deleted = "(D)",
          ignored = "(I)",
        },
      },
    },
  },

  -- Filters configuration (similar to NERDTreeIgnore)
  filters = {
    dotfiles = false,  -- Show hidden files (like g:NERDTreeShowHidden)
    custom = {
      "^\\.ccls-cache$",
      "^\\.pytest_cache$",
      "^__pycache__$",
      "^\\.git$",
    },
  },

  -- Git integration
  git = {
    enable = true,
    ignore = false,  -- Show files even if in .gitignore (like NERDTreeRespectWildIgnore)
    timeout = 500,
  },

  -- Actions configuration
  actions = {
    open_file = {
      quit_on_open = false,
      window_picker = {
        enable = true,
      },
    },
  },

  -- Tab integration
  tab = {
    sync = {
      open = true,
      close = true,
    },
  },
})

-- Keymaps matching NERDTree configuration
-- <C-t> to toggle nvim-tree (matches :NERDTreeTabsToggle)
vim.keymap.set('n', '<C-t>', ':NvimTreeToggle<CR>', { noremap = true, silent = true, desc = "Toggle nvim-tree" })

-- <C-f> to find current file in tree (matches :NERDTreeTabsFind)
vim.keymap.set('n', '<C-f>', ':NvimTreeFindFile<CR>', { noremap = true, silent = true, desc = "Find current file in nvim-tree" })

-- Custom highlight for untracked files - use a more readable red/orange
-- Use autocmd to ensure it applies after colorscheme loads
vim.api.nvim_create_autocmd("ColorScheme", {
  pattern = "*",
  callback = function()
    vim.api.nvim_set_hl(0, 'NvimTreeGitNew', { fg = '#cb4b16' })  -- Solarized orange (brighter red)
    vim.api.nvim_set_hl(0, 'NvimTreeGitNewIcon', { fg = '#cb4b16' })
  end,
})
-- Also set it immediately
vim.api.nvim_set_hl(0, 'NvimTreeGitNew', { fg = '#cb4b16' })  -- Solarized orange (brighter red)
vim.api.nvim_set_hl(0, 'NvimTreeGitNewIcon', { fg = '#cb4b16' })
