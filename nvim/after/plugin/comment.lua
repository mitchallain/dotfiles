local ft = require('Comment.ft')
local calculate = ft.calculate

-- Neovim 0.12 returns nil for a missing Tree-sitter parser rather than throwing.
-- Comment.nvim only checks pcall's success flag, then tries to use that nil tree.
-- Preserve its filetype fallback without disabling embedded-language detection.
ft.calculate = function(ctx)
  local ok, parser = pcall(vim.treesitter.get_parser, vim.api.nvim_get_current_buf())
  if not ok or not parser then
    return ft.get(vim.bo.filetype, ctx.ctype)
  end
  return calculate(ctx)
end

require('Comment').setup()
