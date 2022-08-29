-- print('hello from lua')

-- Inserted from https://github.com/neovim/nvim-lspconfig#suggested-configuration
-- Mappings.
-- See `:help vim.diagnostic.*` for documentation on any of the below functions
local opts = { noremap = true, silent = true }
vim.keymap.set("n", "<leader>di", vim.diagnostic.open_float, opts)
vim.keymap.set("n", "[d", vim.diagnostic.goto_prev, opts)
vim.keymap.set("n", "]d", vim.diagnostic.goto_next, opts)
vim.keymap.set("n", "<leader>q", vim.diagnostic.setloclist, opts)

-- Use an on_attach function to only map the following keys
-- after the language server attaches to the current buffer
local on_attach = function(client, bufnr)
    -- print('LSP attached - loading keymaps')
    -- Enable completion triggered by <c-x><c-o>
    vim.api.nvim_buf_set_option(bufnr, "omnifunc", "v:lua.vim.lsp.omnifunc")

    -- Mappings.
    -- See `:help vim.lsp.*` for documentation on any of the below functions
    local bufopts = { noremap = true, silent = true, buffer = bufnr }
    vim.keymap.set("n", "gD", vim.lsp.buf.declaration, bufopts)
    vim.keymap.set("n", "gd", vim.lsp.buf.definition, bufopts)
    vim.keymap.set("n", "K", vim.lsp.buf.hover, bufopts)
    vim.keymap.set("n", "gi", vim.lsp.buf.implementation, bufopts)
    vim.keymap.set("n", "<leader>k", vim.lsp.buf.signature_help, bufopts)
    vim.keymap.set("n", "<leader>wa", vim.lsp.buf.add_workspace_folder, bufopts)
    vim.keymap.set("n", "<leader>wr", vim.lsp.buf.remove_workspace_folder, bufopts)
    vim.keymap.set("n", "<leader>wl", function()
        print(vim.inspect(vim.lsp.buf.list_workspace_folders()))
    end, bufopts)
    vim.keymap.set("n", "<leader>D", vim.lsp.buf.type_definition, bufopts)
    vim.keymap.set("n", "<leader>rn", vim.lsp.buf.rename, bufopts)
    vim.keymap.set("n", "<leader>ca", vim.lsp.buf.code_action, bufopts)
    vim.keymap.set("n", "gr", vim.lsp.buf.references, bufopts)
    vim.keymap.set("n", "<leader>fo", vim.lsp.buf.formatting, bufopts)
    vim.keymap.set("v", "<leader>fo", vim.lsp.buf.range_formatting, bufopts) -- this isn't quite working yet
end

-- luasnip setup
local luasnip = require("luasnip")

-- nvim-cmp setup
-- https://github.com/hrsh7th/nvim-cmp
local cmp = require("cmp")
cmp.setup({
    snippet = {
        expand = function(args)
            luasnip.lsp_expand(args.body)
        end,
    },
    mapping = cmp.mapping.preset.insert({
        ["<C-d>"] = cmp.mapping.scroll_docs(-4),
        ["<C-f>"] = cmp.mapping.scroll_docs(4),
        ["<C-Space>"] = cmp.mapping.complete(),
        -- ['<CR>'] = cmp.mapping.confirm {
        --   behavior = cmp.ConfirmBehavior.Replace,
        --   select = true,
        -- },
        ["<Tab>"] = cmp.mapping(function(fallback)
            if cmp.visible() then
                cmp.select_next_item()
            elseif luasnip.expand_or_jumpable() then
                luasnip.expand_or_jump()
            else
                fallback()
            end
        end, { "i", "s" }),
        ["<S-Tab>"] = cmp.mapping(function(fallback)
            if cmp.visible() then
                cmp.select_prev_item()
            elseif luasnip.jumpable(-1) then
                luasnip.jump(-1)
            else
                fallback()
            end
        end, { "i", "s" }),
    }),
    sources = {
        { name = "nvim_lsp" },
        { name = "luasnip" },
        { name = "buffer" },
        { name = "cmp_pandoc" },
    },
})

-- Use buffer source for `/` (if you enabled `native_menu`, this won't work anymore).
-- cmp.setup.cmdline('/', {
--   mapping = cmp.mapping.preset.cmdline(),
--   sources = {
--     { name = 'buffer' }
--   }
-- })

-- Use cmdline & path source for ':' (if you enabled `native_menu`, this won't work anymore).
-- cmp.setup.cmdline(':', {
--   mapping = cmp.mapping.preset.cmdline(),
--   sources = cmp.config.sources({
--     { name = 'path' }
--   }, {
--     { name = 'cmdline' }
--   })
-- })

local lsp_flags = {
    -- This is the default in Nvim 0.7+
    debounce_text_changes = 150,
}

-- From https://github.com/neovim/nvim-lspconfig/wiki/Autocompletion#nvim-cmp
-- Add additional capabilities supported by nvim-cmp
local capabilities = vim.lsp.protocol.make_client_capabilities()
capabilities = require("cmp_nvim_lsp").update_capabilities(capabilities)

local lspconfig = require("lspconfig")

lspconfig["pyright"].setup({
    on_attach = on_attach,
    flags = lsp_flags,
    capabilities = capabilities,
})
lspconfig["ccls"].setup({
    on_attach = on_attach,
    flags = lsp_flags,
    capabilities = capabilities,
})
lspconfig["rust_analyzer"].setup({
    on_attach = on_attach,
    flags = lsp_flags,
    capabilities = capabilities,
    -- Server-specific settings...
    settings = {
        ["rust-analyzer"] = {},
    },
})

local null_ls = require("null-ls")

null_ls.setup({
    sources = {
        -- null_ls.builtins.code_actions.gitsigns,
        -- null_ls.builtins.completion.spell,
        null_ls.builtins.diagnostics.markdownlint,
        -- null_ls.builtins.diagnostics.pylint,
        null_ls.builtins.diagnostics.cppcheck,
        null_ls.builtins.diagnostics.flake8,
        null_ls.builtins.formatting.clang_format,

        -- had to npm install --global prettier
        null_ls.builtins.formatting.prettier.with({
            extra_args = { "--print-width", "80", "--prose-wrap", "always" },
            filetypes = { "html", "json", "yaml", "markdown", "xml" }, -- overwrite to add xml via plugin
        }),
        null_ls.builtins.formatting.stylua.with({
            extra_args = { "--indent-type", "Spaces" },
        }),
        null_ls.builtins.formatting.yapf,
    },
    on_attach = on_attach,
    log_level = "warn",
})

-- local diagnostics_active = true
-- local toggle_diagnostics = function()
--     diagnostics_active = not diagnostics_active
--     if diagnostics_active then
--         print("Showing diagnostics")
--         vim.diagnostic.show()
--     else
--         print("Hiding diagnostics")
--         vim.diagnostic.hide()
--     end
-- end
-- vim.keymap.set("n", "<leader>dd", toggle_diagnostics)

-- Plug-in allows easy toggling of diagnostics features
-- start_on currently does not work
require'toggle_lsp_diagnostics'.init({ start_on = false })
require("toggle_lsp_diagnostics").init({
    severity_sort = true,
    underline = true,
    update_in_insert = false,
    virtual_text = {
        spacing = 4,
        prefix = "●",
    },
    float = {
        source = "always",
    },
})
vim.keymap.set("n", "<leader>dd", "<Plug>(toggle-lsp-diag)")
vim.keymap.set("n", "<leader>dt", "<Plug>(toggle-lsp-diag-vtext)")
vim.keymap.set("n", "<leader>de", vim.diagnostic.enable)

-- Diagnostic symbols in the sign column (gutter)
-- local signs = { Error = " ", Warn = " ", Hint = " ", Info = " " }
-- for type, icon in pairs(signs) do
--   local hl = "DiagnosticSign" .. type
--   vim.fn.sign_define(hl, { text = icon, texthl = hl, numhl = "" })
-- end


vim.diagnostic.config({
    severity_sort = true,
    underline = true,
    update_in_insert = false,
    virtual_text = {
        spacing = 4,
        prefix = "●",
    },
    float = {
        source = "always",
    },
})

-- require("colorbuddy").setup()

-- from https://github.com/craftzdog/dotfiles-public/blob/1c58c37e96cd5f451d7f43ac1a3b3c5807752ad9/.config/nvim/after/plugin/neosolarized.rc.lua
require("neosolarized").setup({
    comment_italics = true,
})
