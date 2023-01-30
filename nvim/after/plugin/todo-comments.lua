require("todo-comments").setup {
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
        test = { "Identifier", "#FF00FF" }
    },
    highlight = {
        keyword = "wide_fg",
    },
}
