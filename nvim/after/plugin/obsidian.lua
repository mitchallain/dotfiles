require("obsidian").setup({
    workspaces = {
        {
            name = "work",
            path = "~/notes",
        },
    },
    daily_notes = {
        folder = "03-daily/2025",
        date_format = "%Y-%m-%d",
        template = "00-meta/templates/daily-notes.md",
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
    mappings = {
        -- Overrides the 'gf' mapping to work on markdown/wiki links within your vault.
        ["gf"] = {
            action = function()
                return require("obsidian").util.gf_passthrough()
            end,
            opts = { noremap = false, expr = true, buffer = true },
        },
        -- Smart action depending on context, either follow link or toggle checkbox.
        ["<cr>"] = {
            action = function()
                return require("obsidian").util.smart_action()
            end,
            opts = { buffer = true, expr = true },
        },
    },
})

local opts = { noremap = true, silent = true }
vim.keymap.set("n", "<leader>jj", "<cmd>ObsidianToday<cr>", opts)
vim.keymap.set("n", "<leader>jk", "<cmd>ObsidianYesterday<cr>", opts)
