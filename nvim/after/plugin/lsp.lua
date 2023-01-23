-- Inserted from https://github.com/neovim/nvim-lspconfig#suggested-configuration
-- Mappings.
-- See `:help vim.diagnostic.*` for documentation on any of the below functions
local opts = { noremap = true, silent = true }
vim.keymap.set("n", "<leader>di", vim.diagnostic.open_float, opts)
vim.keymap.set("n", "[d", vim.diagnostic.goto_prev, opts)
vim.keymap.set("n", "]d", vim.diagnostic.goto_next, opts)

-- Testing trouble.nvim
-- vim.keymap.set("n", "<leader>q", vim.diagnostic.setloclist, opts)
vim.keymap.set("n", "<leader>q", "<cmd>TroubleToggle document_diagnostics<cr>", opts)

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
    vim.keymap.set("n", "gs", "<cmd>ClangdSwitchSourceHeader<cr>", opts)

    if client.server_capabilities.hoverProvider then
        vim.keymap.set("n", "K", vim.lsp.buf.hover, bufopts)
    end

    vim.keymap.set("n", "gi", vim.lsp.buf.implementation, bufopts)
    vim.keymap.set("n", "<leader>k", vim.lsp.buf.signature_help, bufopts)

    -- these may be useful but I need to change the keymaps so that <leader>w
    -- is processed immediately for writing
    -- vim.keymap.set("n", "<leader>wa", vim.lsp.buf.add_workspace_folder, bufopts)
    -- vim.keymap.set("n", "<leader>wr", vim.lsp.buf.remove_workspace_folder, bufopts)
    -- vim.keymap.set("n", "<leader>wl", function()
    --     print(vim.inspect(vim.lsp.buf.list_workspace_folders()))
    -- end, bufopts)

    vim.keymap.set("n", "<leader>D", vim.lsp.buf.type_definition, bufopts)
    vim.keymap.set("n", "<leader>rn", vim.lsp.buf.rename, bufopts)
    vim.keymap.set("n", "<leader>ca", vim.lsp.buf.code_action, bufopts)

    -- Testing trouble.nvim
    vim.keymap.set("n", "<leader>sr", vim.lsp.buf.references, bufopts)
    vim.keymap.set("n", "gr", "<cmd>TroubleToggle lsp_references<cr>", bufopts)

    -- Format whole file asynchronously
    vim.keymap.set("n", "<leader>fo", function () vim.lsp.buf.format { async = true } end, bufopts)

    -- Format a range from visual mode
    vim.keymap.set("v", "<leader>fo", vim.lsp.buf.format, bufopts)
end

-- https://github.com/hrsh7th/nvim-cmp/wiki/Example-mappings#luasnip
local has_words_before = function()
    local line, col = unpack(vim.api.nvim_win_get_cursor(0))
    return col ~= 0 and vim.api.nvim_buf_get_lines(0, line - 1, line, true)[1]:sub(col, col):match("%s") == nil
end

-- luasnip setup
local luasnip = require("luasnip")

-- nvim-cmp setup with lspkind
-- https://github.com/hrsh7th/nvim-cmp
local lspkind = require("lspkind")
lspkind.init()

local cmp = require("cmp")
cmp.setup({
    snippet = {
        expand = function(args)
            luasnip.lsp_expand(args.body)
        end,
    },
    mapping = {
        ["<C-n>"] = cmp.mapping(function(fallback)
            if cmp.visible() then
                cmp.select_next_item()
            elseif luasnip.expand_or_jumpable() then
                luasnip.expand_or_jump()
            elseif has_words_before() then
                cmp.complete()
            else
                fallback()
            end
        end, { "i", "s" }),

        ["<C-p>"] = cmp.mapping(function(fallback)
            if cmp.visible() then
                cmp.select_prev_item()
            elseif luasnip.jumpable(-1) then
                luasnip.jump(-1)
            else
                fallback()
            end
        end, { "i", "s" }),

        -- ["<C-n>"] = cmp.mapping.select_next_item({ behavior = cmp.SelectBehavior.Insert }),
        -- ["<C-p>"] = cmp.mapping.select_prev_item({ behavior = cmp.SelectBehavior.Insert }),

        ["<C-d>"] = cmp.mapping.scroll_docs(-4),
        ["<C-f>"] = cmp.mapping.scroll_docs(4),
        ["<C-e>"] = cmp.mapping.abort(),
        ["<C-y>"] = cmp.mapping(
            cmp.mapping.confirm({
                behavior = cmp.ConfirmBehavior.Insert,
                select = true,
            }),
            { "i", "c" }
        ),

        ["<tab>"] = cmp.config.disable,
    },
    sources = {
        -- note that order here dictates priority
        { name = "nvim_lua" },
        { name = "isort" },
        { name = "nvim_lsp" },
        { name = "path" },
        { name = "luasnip" },
        { name = "buffer", keyword_length = 5 },
        { name = "cmp_pandoc" },
    },
    formatting = {
        format = lspkind.cmp_format({
            with_text = true,
            menu = {
                buffer = "[buf]",
                nvim_lsp = "[lsp]",
                nvim_lua = "[api]",
                path = "[path]",
                luasnip = "[snip]",
                gh_issues = "[issues]",
                -- tn = "[tabnine]",
            },
        }),
    },

    experimental = {
        native_menu = false,
        ghost_text = true,
    },
})

-- Use buffer source for `/` (if you enabled `native_menu`, this won't work anymore).
-- cmp.setup.cmdline('/', {
--   mapping = cmp.mapping.preset.cmdline(),
--   sources = {
--     { name = 'buffer', keyword_length = 5 }
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
local capabilities = require("cmp_nvim_lsp").default_capabilities()
-- see https://github.com/jose-elias-alvarez/null-ls.nvim/issues/428#issuecomment-997226723
capabilities.offsetEncoding = { "utf-16" }

local lspconfig = require("lspconfig")

lspconfig["pyright"].setup({
    on_attach = on_attach,
    flags = lsp_flags,
    capabilities = capabilities,
})
lspconfig["clangd"].setup({
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
lspconfig["sumneko_lua"].setup({
    on_attach = on_attach,
    flags = lsp_flags,
    capabilities = capabilities,
    settings = {
        Lua = {
            runtime = {
                -- Tell the language server which version of Lua you're using (most likely LuaJIT in the case of Neovim)
                version = 'LuaJIT',
            },
            diagnostics = {
                -- Get the language server to recognize the `vim` global
                globals = { 'vim' },
            },
            workspace = {
                -- Make the server aware of Neovim runtime files
                library = vim.api.nvim_get_runtime_file("", true),
                checkThirdParty = false,
            },
            -- Do not send telemetry data containing a randomized but unique identifier
            telemetry = {
                enable = false,
            },
        }
    },
})

local null_ls = require("null-ls")

null_ls.setup({
    sources = {
        -- null_ls.builtins.code_actions.gitsigns,
        -- null_ls.builtins.completion.spell,
        null_ls.builtins.diagnostics.markdownlint,
        -- null_ls.builtins.diagnostics.pylint,
        null_ls.builtins.diagnostics.cmake_lint,
        null_ls.builtins.diagnostics.cppcheck,
        null_ls.builtins.diagnostics.flake8,
        -- null_ls.builtins.formatting.clang_format,
        null_ls.builtins.formatting.cmake_format,

        -- had to npm install --global prettier
        null_ls.builtins.formatting.prettier.with({
            extra_args = { "--print-width", "80", "--prose-wrap", "always" },
            filetypes = { "html", "json", "yaml", "markdown", "xml" }, -- overwrite to add xml via plugin
        }),
        null_ls.builtins.formatting.stylua.with({
            extra_args = { "--indent-type", "Spaces" },
        }),
        -- null_ls.builtins.formatting.yapf,
        null_ls.builtins.formatting.black.with({
            extra_args = { "--line-length", "99" },
        }),
    },
    on_attach = on_attach,
    log_level = "warn",
    -- log_level = "debug",
})

local format_diagnostic = function(diagnostic)
    return string.format("%s (%s)", diagnostic.message, diagnostic.code)
end

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
        format = format_diagnostic,
    },
})

-- Plug-in allows easy toggling of diagnostics features
-- start_on currently does not work
require("toggle_lsp_diagnostics").init({
    start_on = true,
    severity_sort = true,
    underline = true,
    update_in_insert = false,
    virtual_text = {
        spacing = 4,
        prefix = "●",
    },
    float = {
        source = "always",
        format = format_diagnostic,
    },
})

vim.keymap.set("n", "<leader>dd", "<Plug>(toggle-lsp-diag)")
vim.keymap.set("n", "<leader>dt", "<Plug>(toggle-lsp-diag-vtext)")
vim.keymap.set("n", "<leader>de", vim.diagnostic.enable)


require('lspfuzzy').setup {}
