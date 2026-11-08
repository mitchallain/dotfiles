local ok, persistence = pcall(require, "persistence")
if not ok then
  return
end

persistence.setup({
  branch = false,
})

-- Start disabled during startup so `+qall`, `--headless`, or `nvim <file>`
-- cannot trigger VimLeavePre before VimEnter confirms a bare interactive launch.
if vim.v.vim_did_enter == 0 then
  persistence.stop()
end

-- Ignore unlisted buffers (e.g. ShaDa global marks A-Z) when deciding whether
-- to save a session, so quitting an empty nvim doesn't overwrite a valid session.
local orig_save = persistence.save
persistence.save = function()
  for _, b in ipairs(vim.api.nvim_list_bufs()) do
    if
      vim.bo[b].buflisted
      and vim.bo[b].buftype == ""
      and not vim.tbl_contains({ "gitcommit", "gitrebase", "jj" }, vim.bo[b].filetype)
      and vim.api.nvim_buf_get_name(b) ~= ""
    then
      orig_save()
      return
    end
  end
end

local group = vim.api.nvim_create_augroup("mallain_persistence", { clear = true })

-- Close nvim-tree before :mksession so NvimTree_1 isn't saved as an empty buffer
vim.api.nvim_create_autocmd("User", {
  group = group,
  pattern = "PersistenceSavePre",
  callback = function()
    pcall(vim.cmd, "NvimTreeClose")
  end,
})

-- Track stdin reads (e.g. `cmd | nvim -`) so we don't auto-restore over piped input
vim.api.nvim_create_autocmd("StdinReadPre", {
  group = group,
  callback = function()
    vim.g.started_with_stdin = true
  end,
})

vim.api.nvim_create_user_command("PersistenceLoad", function()
  persistence.start()
  if vim.v.vim_did_enter == 1 then
    persistence.load()
  else
    vim.g.persistence_restore = true
  end
end, { desc = "Restore session for cwd" })

-- Enable session saving only when starting a bare interactive `nvim`
-- (no file args, no stdin, not nested, not headless, no +cmd buffers/windows)
-- so one-off `nvim <file>`, `nvim +PlugInstall`, or `nvim --headless`
-- invocations don't overwrite the saved project session.
-- Only auto-restore when launched with `+PersistenceLoad` (by tmux-resurrect).
vim.api.nvim_create_autocmd("VimEnter", {
  group = group,
  nested = true,
  callback = function()
    local is_bare_interactive = vim.fn.argc() == 0
      and not vim.g.started_with_stdin
      and not vim.env.NVIM
      and #vim.api.nvim_list_uis() > 0
      and #vim.api.nvim_list_wins() == 1
      and vim.api.nvim_buf_get_name(0) == ""
      and vim.bo[0].buftype == ""
      and vim.fn.line2byte("$") == -1

    if is_bare_interactive then
      persistence.start()
      if vim.g.persistence_restore then
        persistence.load()
      end
    end
  end,
})

-- Keymaps for manual session control
vim.keymap.set("n", "<leader>qs", function() persistence.start(); persistence.load() end, { desc = "Restore session for cwd" })
vim.keymap.set("n", "<leader>qS", function() persistence.start(); persistence.select() end, { desc = "Select session" })
vim.keymap.set("n", "<leader>ql", function() persistence.start(); persistence.load({ last = true }) end, { desc = "Restore last session" })
vim.keymap.set("n", "<leader>qd", function() persistence.stop() end, { desc = "Don't save current session" })
