-- nabla.nvim configuration
-- Floating LaTeX equation renderer for markdown files

-- Keymaps for markdown files
vim.api.nvim_create_autocmd("FileType", {
  pattern = "markdown",
  callback = function()
    -- Show LaTeX equation in floating window
    vim.keymap.set('n', '<leader>p', function()
      require("nabla").popup()
    end, { buffer = true, desc = "Show LaTeX equation popup" })

    -- Enable virtual text rendering of equations
    vim.keymap.set('n', '<leader>pe', function()
      require("nabla").enable_virt()
    end, { buffer = true, desc = "Enable LaTeX virtual text" })

    -- Disable virtual text rendering of equations
    vim.keymap.set('n', '<leader>pd', function()
      require("nabla").disable_virt()
    end, { buffer = true, desc = "Disable LaTeX virtual text" })
  end,
})
