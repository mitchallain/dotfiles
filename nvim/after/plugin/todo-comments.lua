require("todo-comments").setup {
    keywords = {
        TODO = { icon = " ", color = "todo" },
    },
    -- gui_style = {
    --     fg = "NONE",
    --     bg = "BOLD",
    -- },
    colors = {
        error = { "DiagnosticError", "ErrorMsg", "#DC2626" },
        warning = { "DiagnosticWarn", "WarningMsg", "#FBBF24" },
        info = { "DiagnosticInfo", "#2563EB" },
        hint = { "DiagnosticHint", "#10B981" },
        default = { "Identifier", "#7C3AED" },
        test = { "Identifier", "#FF00FF" },
        todo = { "TODO", "#d33682" }
    },
    highlight = {
        keyword = "wide_fg",
        -- Anduril clang-tidy requires usernames, below is vim script regex
        -- https://github.com/folke/todo-comments.nvim/issues/10#issuecomment-886059258
        pattern = [[.*<(KEYWORDS)(\([^\)]*\))?:]],
    },
    search = {
        -- Below is rust regex for ripgrep
        -- https://github.com/folke/todo-comments.nvim/issues/10#issuecomment-886059258
        pattern = [[\b(KEYWORDS)(\([^\)]*\))?:]],
    },
}
