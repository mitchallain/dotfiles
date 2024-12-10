require("obsidian").setup({
    workspaces = {
        {
            name = "work",
            path = "~/notes",
        },
    },
    daily_notes = {
        folder = "03-daily/2024",
        date_format = "%Y-%m-%d",
        template = "00-meta/templates/daily-notes.md"
    },
    templates = {
        folder = "00-meta/templates",
        date_format = "%Y-%m-%d",
        time_format = "%H:%M",
    },
    ui = {
        enable = false,
    },
    disable_frontmatter = true,
})

local opts = { noremap = true, silent = true }
vim.keymap.set("n", "<leader>jj", "<cmd>ObsidianToday<cr>", opts)
vim.keymap.set("n", "<leader>jk", "<cmd>ObsidianYesterday<cr>", opts)
